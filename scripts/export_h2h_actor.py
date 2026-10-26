"""Export the deterministic H1 H2H Actor from a training checkpoint to TorchScript."""
import argparse
from collections import OrderedDict
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile

import torch
from torch import nn

METADATA_FILE = 'h2h_export_metadata.json'


def build_actor(checkpoint):
    if not isinstance(checkpoint, dict) or not isinstance(checkpoint.get('model_state_dict'), dict):
        raise ValueError('Expected a training checkpoint containing model_state_dict')
    policy = checkpoint.get('training_config', {}).get('policy', {})
    if policy.get('activation') != 'elu':
        raise ValueError('This exporter requires training_config.policy.activation=elu; no activation is guessed')
    hidden = policy.get('actor_hidden_dims')
    if not isinstance(hidden, (list, tuple)) or not hidden or any(type(n) is not int or n <= 0 for n in hidden):
        raise ValueError('Missing or invalid training_config.policy.actor_hidden_dims')
    state = OrderedDict((name[len('actor.'):], value)
                        for name, value in checkpoint['model_state_dict'].items() if name.startswith('actor.'))
    first = state.get('0.weight')
    last = state.get(f'{2 * len(hidden)}.weight')
    if first is None or last is None or first.ndim != 2 or last.ndim != 2:
        raise ValueError('Expected a feedforward Linear/ELU Actor state dictionary')
    if first.shape[1] != 1665 or last.shape[0] != 19:
        raise ValueError(f'H1 H2H contract requires 1665 inputs and 19 outputs, got {first.shape[1]} and {last.shape[0]}')
    widths = [1665, *hidden, 19]
    layers = []
    for i, (inputs, outputs) in enumerate(zip(widths, widths[1:])):
        layers.append(nn.Linear(inputs, outputs))
        if i < len(hidden):
            layers.append(nn.ELU())
    actor = nn.Sequential(*layers).eval()
    actor.load_state_dict(state, strict=True)
    if not all(torch.isfinite(value).all() for value in actor.state_dict().values()):
        raise ValueError('Actor has non-finite weights')
    actor.requires_grad_(False)
    return actor, hidden


def export_actor(checkpoint_path, output_path):
    source, output = Path(checkpoint_path).resolve(), Path(output_path).resolve()
    if source == output:
        raise ValueError('Output must differ from the original checkpoint')
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}; choose a new output path')
    if zipfile.is_zipfile(source):
        with zipfile.ZipFile(source) as archive:
            if any('/code/' in name for name in archive.namelist()):
                raise ValueError('Input is already TorchScript; expected a training checkpoint')
    checkpoint = torch.load(source, map_location='cpu', weights_only=True)
    actor, hidden = build_actor(checkpoint)
    scripted = torch.jit.script(actor)
    generator = torch.Generator().manual_seed(0)
    probes = (torch.zeros(1, 1665), torch.randn(32, 1665, generator=generator))
    metadata = dict(format='h1-h2h-actor-v1', source_file=source.name,
                    source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                    iteration=checkpoint.get('iter'), input_size=1665, output_size=19,
                    actor_hidden_dims=list(hidden), activation='elu', deterministic_actor=True,
                    note='Network export only; observation semantics and control parameters are not certified.')
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(prefix='.h2h-actor-', suffix='.pt', dir=output.parent, delete=False) as stream:
        temporary = Path(stream.name)
    try:
        torch.jit.save(scripted, str(temporary), _extra_files={METADATA_FILE: json.dumps(metadata)})
        loaded = torch.jit.load(str(temporary), map_location='cpu').eval()
        max_error = 0.
        with torch.no_grad():
            for probe in probes:
                original, restored = actor(probe), loaded(probe)
                if restored.shape != (probe.shape[0], 19) or not torch.isfinite(restored).all():
                    raise ValueError('Exported Actor failed the finite output/shape check')
                torch.testing.assert_close(original, restored, rtol=1e-6, atol=1e-6)
                max_error = max(max_error, float((original-restored).abs().max()))
        # No overwrite, including if another process created the destination
        # during validation. Both files live on the same filesystem.
        os.link(temporary, output)
    finally:
        temporary.unlink(missing_ok=True)
    return dict(**metadata, output=str(output), equivalence_max_abs_error=max_error,
                verification='CPU zero input and 32 deterministic random inputs, before/after serialization')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    try:
        result = export_actor(args.checkpoint, args.output)
    except (ValueError, FileExistsError) as error:
        parser.error(str(error))
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()

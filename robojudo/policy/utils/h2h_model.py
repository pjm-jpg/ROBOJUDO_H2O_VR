"""Shared model selection for the Quest simulation and H1 deployment launchers."""
import argparse
import json
import logging
import re
import zipfile
from pathlib import Path


logger = logging.getLogger(__name__)

# This exact checkpoint was matched to studentmodel/env_cfg.json by SHA256.
# This checkpoint expects pelvis observations; legacy H1 deployment uses torso.
# Training control consumes Actor actions without the deployment EMA filter.
# Do not apply this to a model just because it has the same filename or shape.
MODEL_OBSERVATION_PROFILES = {
    ('h1', 'c18868aab143f15e29d07fa887da99620e74b3feaafc3c768000eaa253359519'):
        {'use_imu_torso': False, 'action_beta': 1.0},
}


def policy_name(value):
    name = value[:-3] if value.endswith('.pt') else value
    if not name or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]*', name) or '..' in name:
        raise argparse.ArgumentTypeError('Use a model name such as model_30000_jit, without a directory')
    return name


def select_h2h_model(config, name):
    selected = type(config).model_validate({**config.model_dump(), 'policy_name': policy_name(name)})
    path = Path(selected.policy_file)
    if not path.is_file():
        raise FileNotFoundError(f'H2H policy file not found: {path}')
    if not zipfile.is_zipfile(path):
        raise ValueError(f'{path.name} is not a TorchScript archive; export the Actor first')
    with zipfile.ZipFile(path) as archive:
        if not any('/code/' in member for member in archive.namelist()):
            raise ValueError(f'{path.name} is a training checkpoint, not an inference model. '
                             'Export its Actor with scripts/export_h2h_actor.py, then select the exported name.')
        metadata_names = [member for member in archive.namelist()
                          if member.endswith('/extra/h2h_export_metadata.json')]
        metadata = json.loads(archive.read(metadata_names[0])) if metadata_names else {}
    overrides = MODEL_OBSERVATION_PROFILES.get((selected.robot, metadata.get('source_sha256')))
    if overrides:
        selected = type(config).model_validate({**selected.model_dump(), **overrides})
        logger.info('H2H Actor %s: matched checkpoint profile; IMU source=pelvis, action_beta=%s',
                    path.name, selected.action_beta)
    return selected

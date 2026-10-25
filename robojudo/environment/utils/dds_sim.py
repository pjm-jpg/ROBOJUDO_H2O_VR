"""Local-only DDS setup and simulator lifecycle channel (not a robot protocol)."""
import json

DEFAULT_DOMAIN = 42
CONTROL_TOPIC = 'robojudo/sim/control'
STATUS_TOPIC = 'robojudo/sim/status'
TARGET_TOPIC = 'robojudo/sim/targets'
_initialized_domain = None


def initialize_sim_dds(domain=DEFAULT_DOMAIN):
    global _initialized_domain
    if not 1 <= domain <= 100:
        raise ValueError('Simulation DDS domain must be between 1 and 100; domain 0 is reserved for hardware')
    if _initialized_domain is not None:
        if domain != _initialized_domain:
            raise RuntimeError('DDS is already initialized with another domain')
        return
    from unitree_sdk2py.core import channel
    # SDK normally selects an interface but leaves discovery implicit. Explicit
    # localhost peers also work on lo, where multicast is usually disabled.
    channel.ChannelConfigHasInterface = '''<CycloneDDS><Domain Id="any">
      <General><Interfaces><NetworkInterface name="lo"/></Interfaces>
      <AllowMulticast>false</AllowMulticast></General>
      <Discovery><ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>30</MaxAutoParticipantIndex>
      <Peers><Peer Address="127.0.0.1"/></Peers></Discovery>
    </Domain></CycloneDDS>'''
    factory = channel.ChannelFactory()
    if getattr(factory, '_ChannelFactory__initialized', False):
        raise RuntimeError('Run DDS simulator in a fresh process; a channel factory already exists')
    channel.ChannelFactoryInitialize(domain, 'lo')
    _initialized_domain = domain


def decode_status(message):
    value = json.loads(message.data)
    if value.get('protocol') != 'robojudo-mujoco-h1-v1':
        raise ValueError('Unexpected simulator protocol')
    return value

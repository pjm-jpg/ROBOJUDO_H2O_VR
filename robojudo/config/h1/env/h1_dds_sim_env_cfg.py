from pydantic import model_validator

from .h1_real_env_cfg import H1RealEnvCfg, H1UnitreeCfg


class H1DDSSimEnvCfg(H1RealEnvCfg):
    env_type: str = 'UnitreeDDSSimEnv'
    is_sim: bool = True
    unitree: H1UnitreeCfg = H1UnitreeCfg(net_if='lo', domain_id=42)
    born_place_align: bool = False

    @model_validator(mode='after')
    def local_only(self):
        if (self.unitree.net_if != 'lo' or not 1 <= self.unitree.domain_id <= 100
                or self.unitree.robot != 'h1' or self.unitree.msg_type != 'go'):
            raise ValueError('H1 DDS simulation requires lo, nonzero domain 1..100 and H1 go messages')
        if not self.is_sim or not self.act:
            raise ValueError('DDS simulation requires is_sim=True and act=True')
        return self

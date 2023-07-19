import asyncio
import sys

from viam.components.base import Base
from viam.module.module import Module
from components import RosBase
from utils import RclpyNodeManager

async def main(addr: str) -> None:
    m = Module(addr)
    m.add_model_from_registry(Base.SUBTYPE, RosBase.MODEL)
    await m.start()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    rclpyMgr = RclpyNodeManager.get_instance()

    asyncio.run(main(sys.argv[1]))
    rclpyMgr.shutdown()

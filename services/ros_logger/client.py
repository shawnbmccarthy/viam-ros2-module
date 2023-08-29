import asyncio

from .my_ros_logger import MyRosLoggerService

from viam import logging
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions


async def connect():
    creds = Credentials(type="<your authentication type here>", payload="<your authentication payload here>")
    opts = RobotClient.Options(refresh_interval=0, dial_options=DialOptions(credentials=creds), log_level=logging.DEBUG)
    return await RobotClient.at_address("<your robot uri here>", opts)


async def main():
    robot = await connect()

    print("Resources:")
    print(robot.resource_names)

    # # ####### ROS LOGGER ####### #
    summer = MyRosLoggerService.from_robot(robot, name="mysum1")
    sum = await summer.sum([0, 1, 2, 3, 4, 5, 6, 7, 8, 9])
    print(f"The sum of the numbers [0, 10) is {sum}")

    await robot.close()


if __name__ == "__main__":
    asyncio.run(main())
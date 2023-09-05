# Generated by the Protocol Buffers compiler. DO NOT EDIT!
# source: proto/ros2_logger.proto
# plugin: grpclib.plugin.main
import abc
import typing

import grpclib.const
import grpclib.client
if typing.TYPE_CHECKING:
    import grpclib.server

import google.api.annotations_pb2
import proto.ros2_logger_pb2


class ROS2LoggerServiceBase(abc.ABC):

    @abc.abstractmethod
    async def Status(self, stream: 'grpclib.server.Stream[proto.ros2_logger_pb2.Request, proto.ros2_logger_pb2.Response]') -> None:
        pass

    def __mapping__(self) -> typing.Dict[str, grpclib.const.Handler]:
        return {
            '/viamlabs.service.ros2_logger.v1.ROS2LoggerService/Status': grpclib.const.Handler(
                self.Status,
                grpclib.const.Cardinality.UNARY_UNARY,
                proto.ros2_logger_pb2.Request,
                proto.ros2_logger_pb2.Response,
            ),
        }


class ROS2LoggerServiceStub:

    def __init__(self, channel: grpclib.client.Channel) -> None:
        self.Status = grpclib.client.UnaryUnaryMethod(
            channel,
            '/viamlabs.service.ros2_logger.v1.ROS2LoggerService/Status',
            proto.ros2_logger_pb2.Request,
            proto.ros2_logger_pb2.Response,
        )
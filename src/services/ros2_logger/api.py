"""
This file outlines the general structure for the API around a custom, modularized service.

It defines the abstract class definition that all concrete implementations must follow,
the gRPC service that will handle calls to the service,
and the gRPC client that will be able to make calls to this service.

In this example, the ``ROS2LoggerService`` abstract class defines what functionality is required for all ``ros2_logger`` services.
It extends ``ServiceBase``, as all service types must.
It also defines its specific ``SUBTYPE``, which is used internally to keep track of supported types.

The ``ROS2LoggerRPCService`` implements the gRPC service for the ROS2LoggerService service. This will allow other robots and clients to make
requests of the ROS2LoggerService service. It extends both from ``ROS2LoggerServiceBase`` and ``RPCServiceBase``.
The former is the gRPC service as defined by the proto, and the latter is the class that all gRPC services must inherit from.

Finally, the ``ROS2LoggerClient`` is the gRPC client for a ROS2LoggerService service. It inherits from ROS2LoggerService since it implements
 all the same functions. The implementations are simply gRPC calls to some remote ROS2LoggerService service.

To see how this custom modular service is registered, see the __init__.py file.
To see the custom implementation of this service, see the ros2_logger.py file.
"""

import abc
from typing import Final, Sequence

from grpclib.client import Channel
from grpclib.server import Stream

from viam.resource.rpc_service_base import ResourceRPCServiceBase
from viam.resource.types import RESOURCE_TYPE_SERVICE, Subtype
from viam.services.service_base import ServiceBase

from proto.ros2_logger_grpc import ROS2LoggerServiceBase, ROS2LoggerServiceStub
from proto.ros2_logger_pb2 import Request, Response


class ROS2LoggerService(ServiceBase):
    """Viam ROS 2 logger service subclass of the ServiceBase class including additional abstract methods to be implemented"""

    SUBTYPE: Final = Subtype("viamlabs", RESOURCE_TYPE_SERVICE, "ros2_logger")

    @abc.abstractmethod
    async def status(self) -> Response:
        ...


class ROS2LoggerRPCService(ROS2LoggerServiceBase, ResourceRPCServiceBase):
    """gRPC service for the ROS2LoggerService"""

    RESOURCE_TYPE = ROS2LoggerService

    async def Status(self, stream: Stream[Request, Response]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.status()
        await stream.send_message(resp)


class ROS2LoggerClient(ROS2LoggerService):
    """gRPC client for the ROS2LoggerService"""

    def __init__(self, name: str, channel: Channel) -> None:
        self.channel = channel
        self.client = ROS2LoggerServiceStub(channel)
        super().__init__(name)

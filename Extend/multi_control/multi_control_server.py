#!/usr/bin/env python3
# This file is part of TonyPi.
# Copyright (C) 2021 Hiwonder Ltd. <support@hiwonder.com>
#
# rsp_robot_hat_v3 is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# rsp_robot_hat_v3 is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# title           :multi_control_server.py
# author          :Hiwonder, LuYongping(Lucas)
# date            :20210421
# notes           : 这是一个websocket中继广播服务器， 接受up连接的数据 发送到 down连接(this is a WebSocket relay broadcasting server, which accepts data from 'up' connections and forwards it to 'down' connections)
# ==============================================================================
import asyncio
import websockets

down_clients = set()  # 已连接的下行客户端(the connected downstream client)

async def broadcaster(socket: websockets.WebSocketClientProtocol, url_path):
    connected = server.ws_server.websockets
    if url_path.endswith('down'):  # 下行连接管理(downstream connection management)
        down_clients.add(socket)
        await socket.wait_closed()
        down_clients.remove(socket)
    elif url_path.endswith('up'):  # 接收上行数据并转发(receive upstream data and forward it)
        async for msg in socket:
            for ws in connected:
                await ws.send(msg)
    else:
        pass

server = websockets.serve(broadcaster, '0.0.0.0', 7788)
asyncio.get_event_loop().run_until_complete(server)
asyncio.get_event_loop().run_forever()


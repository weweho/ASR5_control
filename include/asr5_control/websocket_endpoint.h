#pragma once
#ifndef _WEBSOCKET_ENDPOINT_
#define _WEBSOCKET_ENDPOINT_

#include <string>

/*
Title: Web Socket Client
Author: kagula
Date: 2016-11-21
Dependencies: Websocket++¡¢Boost::ASIO
Test Environment: VS2013 Update5, WebSocket++ 0.70, Boost 1.61
Description:
[1]Support connect a web socket server.
[2]If server is crash, client will not follow crash.
*/

namespace kagula
{
	class websocket_endpoint {
	public:
		websocket_endpoint();
		~websocket_endpoint();

		int connect(std::string const& uri);
		void close();

		void send(std::string message);
		void show();
	};
}

#endif

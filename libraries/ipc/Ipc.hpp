/*
 * MIT License
 *
 * Copyright (c) 2021 Gerald Young (Yoyobuae)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the “Software”),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#pragma once

#include <string>

#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#include <windows.h>
#endif

namespace Ipc {

    class Connection {
        public:
            ~Connection();

            // Copying not allowed
            Connection(Connection const &) = delete;
            Connection& operator=(Connection const &) = delete;

            // Moving is allowed
            Connection(Connection &&) = default;
            Connection& operator=(Connection &&) = default;

            bool send(const char *buffer, size_t length);
            bool recv(char *buffer, size_t length);

        private:
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
            Connection(HANDLE inPipe);
            HANDLE inPipe;
#elif defined(__linux) || defined(__linux__) || defined(linux)
            Connection(int connfd);
            int connfd;
#else
            Connection();
#endif
            friend class Server;
    };

    class Server {
        public:
            Server();

            // Copying not allowed
            Server(Server const &) = delete;
            Server& operator=(Server const &) = delete;

            void init(std::string name);
            Connection accept();

        private:
#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
            HANDLE inPipe;
#elif defined(__linux) || defined(__linux__) || defined(linux)
            int listenfd;
#endif
    };

    class Client {
        public:
            Client(std::string name);

            // Copying not allowed
            Client(Client const &) = delete;
            Client& operator=(Client const &) = delete;

            std::string sendrecv(std::string buffer);

        private:
            std::string name;
    };

}; // namespace Ipc

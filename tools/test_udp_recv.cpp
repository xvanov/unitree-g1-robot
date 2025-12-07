// Simple UDP receiver test - listens on port and prints received packets
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

int main(int argc, char** argv) {
    int port = 5001;
    if (argc > 1) {
        port = std::stoi(argv[1]);
    }

    // Create UDP socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return 1;
    }

    // Set receive timeout (5 seconds)
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Bind to port
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind to port " << port << ": " << strerror(errno) << std::endl;
        close(sock);
        return 1;
    }

    std::cout << "Listening on UDP port " << port << "..." << std::endl;
    std::cout << "Waiting for packets (5 second timeout per recv)..." << std::endl;

    char buf[65536];
    struct sockaddr_in sender;
    socklen_t sender_len = sizeof(sender);

    int count = 0;
    while (count < 10) {
        ssize_t len = recvfrom(sock, buf, sizeof(buf), 0,
                               (struct sockaddr*)&sender, &sender_len);
        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::cout << "Timeout - no packet received" << std::endl;
                continue;
            }
            std::cerr << "Receive error: " << strerror(errno) << std::endl;
            break;
        }

        char sender_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &sender.sin_addr, sender_ip, INET_ADDRSTRLEN);

        std::cout << "Received " << len << " bytes from " << sender_ip
                  << ":" << ntohs(sender.sin_port) << std::endl;

        // Check if it looks like a depth frame packet
        if (len >= 16) {
            uint32_t magic = *reinterpret_cast<uint32_t*>(buf);
            if (magic == 0x44455054) {
                uint32_t frame_num = *reinterpret_cast<uint32_t*>(buf + 4);
                uint16_t chunk_idx = *reinterpret_cast<uint16_t*>(buf + 8);
                uint16_t chunk_count = *reinterpret_cast<uint16_t*>(buf + 10);
                std::cout << "  -> Depth packet: frame " << frame_num
                          << " chunk " << chunk_idx << "/" << chunk_count << std::endl;
            } else {
                std::cout << "  -> Magic: 0x" << std::hex << magic << std::dec << std::endl;
            }
        }

        count++;
    }

    close(sock);
    return 0;
}

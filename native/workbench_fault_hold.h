#pragma once
// Wire layout shared with robot_workbench/native_fault_hold.py.
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace workbench {
class FaultHold {
    uint64_t *page_ = nullptr;
    int side_ = -1;
    static constexpr uint64_t magic = 0x574246484f4c4431ULL;
    uint64_t read(int word) const {
        return __atomic_load_n(page_ + word, __ATOMIC_ACQUIRE);
    }
public:
    FaultHold() = default;
    FaultHold(const FaultHold &) = delete;
    FaultHold &operator=(const FaultHold &) = delete;
    ~FaultHold() { if (page_) munmap(page_, 4096); }
    static uint64_t now() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    void bind(int fd, int side) {
        struct stat info;
        if (page_ || (side != 0 && side != 1) || fstat(fd, &info) ||
            !S_ISREG(info.st_mode) || info.st_uid != getuid() ||
            (info.st_mode & 0777) != 0600 || info.st_size != 4096)
            throw std::invalid_argument("private native fault-hold page required");
        void *mapped = mmap(nullptr, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (mapped == MAP_FAILED) throw std::runtime_error("native fault-hold mmap failed");
        page_ = static_cast<uint64_t *>(mapped);
        if (read(0) != magic) {
            munmap(page_, 4096); page_ = nullptr;
            throw std::invalid_argument("native fault-hold page version differs");
        }
        side_ = side;
    }
    bool fault() const {
        if (!page_ || !read(1)) return false;
        const uint64_t heartbeat = read(2), current = now();
        return read(3) || current < heartbeat || current - heartbeat > 100000000ULL;
    }
    void trip(uint64_t reason) {
        if (!page_ || !read(1)) return;
        uint64_t expected = 0;
        __atomic_compare_exchange_n(page_ + 3, &expected, reason, false,
                                    __ATOMIC_RELEASE, __ATOMIC_RELAXED);
    }
    void committed() {
        if (page_ && !read(4 + side_))
            __atomic_store_n(page_ + 4 + side_, now(), __ATOMIC_RELEASE);
    }
};
}

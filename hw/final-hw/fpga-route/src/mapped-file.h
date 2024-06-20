#ifndef MAPPED_FILE_H
#define MAPPED_FILE_H

#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <sys/mman.h>
#include <unistd.h>

class MappedFile {
public:
  MappedFile(std::string filename) {
    int fd = open(filename.c_str(), O_RDONLY);
    if (fd == -1) {
      throw std::runtime_error("Failed to open file");
    }

    size_ = lseek(fd, 0, SEEK_END);

    data_ = mmap(NULL, size_, PROT_READ, MAP_PRIVATE, fd, 0);
    if (data_ == MAP_FAILED) {
      close(fd);
      throw std::runtime_error("Failed to create memory mapping");
    }

    close(fd);
  }

  MappedFile(const MappedFile &) = delete;
  MappedFile &operator=(const MappedFile &) = delete;

  ~MappedFile() { munmap(data_, size_); }

  const char *data() const { return static_cast<const char *>(data_); }

  size_t size() const { return size_; }

private:
  void *data_;
  size_t size_;
};

#endif // MAPPED_FILE_H
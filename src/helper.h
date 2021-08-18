#include <stddef.h>
#include <stdint.h>

class Node{
public:
  Node* next = NULL;
  const char* name = NULL;
};

#define FD_READ     (1 << 0)
#define FD_WRITE    (1 << 2)

typedef struct{
  int fd;
  int event_mask;
} poll_fd;

class Poller : public Node{
public:
  virtual int getFds(poll_fd* fds){return -1;};
  virtual void OnFDData(poll_fd fd) = 0;
};

Node* addNode(Node* nodes, Node* node);
Node* removeNode(Node* nodes, Node* node);

void hexdump(uint8_t* buffer, uint32_t buff_len, const char* head);

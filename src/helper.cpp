#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "helper.h"

Node* addNode(Node* nodes, Node* node){
  if(!nodes) nodes = node;
  else{
    Node* tail = nodes;
    while(tail){
      if(tail == node) goto end;
      if(!tail->next){
        tail->next = node;
        break;
      }
      tail = tail->next;
    }
  }
  printf("added %s %p\r\n", node->name, node);

  node->next = NULL;
  end:
  return nodes;
}

Node* removeNode(Node* nodes, Node* node){
  if(!node || !nodes) goto end;
  if(node == nodes) nodes = nodes->next;
  else{
    Node* prev = nodes;
    Node* tail = nodes->next;
    while(tail){
      if(tail == node){
        prev->next = node->next;
        break;
      }
      if(!tail->next) goto end;
      prev = tail;
      tail = tail->next;
    }
  }
  printf("removed %s %p\r\n", node->name, node);
  node->next = NULL;
  end:
  return nodes;
}

void *operator new(size_t size){
  return malloc(size);
}

void operator delete(void *p){
  free(p);
}

void *operator new[](size_t size){
  return malloc(size);
}

void operator delete[](void *p){
  free(p);
}

#define BYTES_PER_LINE 16

static char hd_buffer[10 + 3 + (BYTES_PER_LINE * 3) + 3 + BYTES_PER_LINE + 1 + 1];

uint32_t GetMsSincePwrOn(){
  return 0;
}

void hexdump(uint8_t* buffer, uint32_t buff_len, const char* head){
  if(buff_len == 0) return;

  // uint8_t* orgBuffer = buffer;
  uint32_t bytes_cur_line = buff_len > BYTES_PER_LINE ? BYTES_PER_LINE : buff_len;

  printf("%u|%s -> %d bytes\n", GetMsSincePwrOn(), head, buff_len);

  char* ptr_hd = hd_buffer + sprintf(hd_buffer, "%p", buffer);
  for(uint8_t i = 0; i < BYTES_PER_LINE; i++) {
    if ((i&7)==0 ) ptr_hd += sprintf(ptr_hd, " " );
    if (i < bytes_cur_line) ptr_hd += sprintf(ptr_hd, "  %1x", i);
    else ptr_hd += sprintf(ptr_hd, "   ");
  }
  ptr_hd += sprintf(ptr_hd, "   ");
  for(uint8_t i = 0; i < bytes_cur_line; i++) {
    ptr_hd += sprintf(ptr_hd, "%1x", i);
  }
  printf("%s\n", hd_buffer);

  do {
    ptr_hd = hd_buffer;
    bytes_cur_line = buff_len > BYTES_PER_LINE ? BYTES_PER_LINE : buff_len;

    ptr_hd += sprintf(ptr_hd, "%p", buffer);
    for(uint32_t i = 0; i < BYTES_PER_LINE; i++) {
      if ((i&7)==0 ) ptr_hd += sprintf(ptr_hd, " ");
      if (i < bytes_cur_line) ptr_hd += sprintf(ptr_hd, " %02x", buffer[i]);
      else ptr_hd += sprintf(ptr_hd, "   ");
    }

    ptr_hd += sprintf(ptr_hd, "  |");

    for(uint32_t i = 0; i < bytes_cur_line; i++){
      if(isprint(buffer[i])) ptr_hd += sprintf(ptr_hd, "%c", buffer[i]);
      else ptr_hd += sprintf(ptr_hd, ".");
    }

    ptr_hd += sprintf(ptr_hd, "|");

    printf("%s\n", hd_buffer);
    buffer += bytes_cur_line;
    buff_len -= bytes_cur_line;
  } while(buff_len);

  return;
}

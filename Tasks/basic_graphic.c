#include "stm32f4xx_hal.h"
#include "basic_graphic.h"
#define ROBOT_RED_HERO_1 0x001
#define ROBOT_BLUE_HERO_1 0x065

#define TX_IT_SIGNAL_DISABLED 0

static osThreadId_t senderID;

uint8_t gbuf[2][GRAPHIC_CMD_BUFFER_SIZE];
int gbuf_lock[2] = {0,0};
int gbuf_tail[2] = {0,0};
int gbuf_count[2] = {0,0};

static inline int ceiling(int n) {
  switch (n) {
    case 1:
      return 1;
    case 2:
      return 2;
    case 3: case 4: case 5:
      return 5;
    case 6: case 7:
      return 7;
    default:
      return 0;
  }
}

/* this helper function need the buffer be locked prehead */
void sealBuffer(int i) {
  /* if open multi-graph packet at the end of buffer */
  if (gbuf_count[i]) {
    /* final packet number of the packet before */
    int n = ceiling(gbuf_count[i]);
    /* padding bytes number */
    int m = (n - gbuf_count[i]) * 15;
    /* zero padding bytes */
    memset(gbuf[i]+gbuf_tail[i], 0, m);
    /* update buffer tail */
    gbuf_tail[i] += m;
    /* reset graph counter */
    gbuf_count[i] = 0;
    
    /* select correct DCMD for the packet */
    uint8_t dcmd;
    switch (n) {
      case 1:
        dcmd = DCMD_UI_OP_1 & 0xff;
        break;
      case 2:
        dcmd = DCMD_UI_OP_2 & 0xff;
        break;
      case 5:
        dcmd = DCMD_UI_OP_5 & 0xff;
        break;
      case 7:
        dcmd = DCMD_UI_OP_7 & 0xff;
        break;
      default:
        dcmd = 0xff;
    }
    gbuf[i][ gbuf_tail[i]-15*n-6 ] = dcmd;
  }
}

static inline void initHeader(uint8_t* tail, int dcmd) {
  tail[0] = dcmd & 0xff;
  tail[1] = (dcmd >> 8) & 0xff;
  tail[2] = tail[4] = SELF_ID & 0xff;
  /* higher byte of sender ID is always zero */
  tail[3] = 0;
  /* higher byte of receiver ID is 0x01 */
  tail[5] = 1;
}

int sendGraphic(graphic_base_t const * packet, void const * buf) {
  /* try all buffers */
  for (int i = 0; i < 2; ++i) {
    /* switch if this buffer is in use */
    if (gbuf_lock[i]) continue;
    /* lock the buffer */
    gbuf_lock[i] = 1;
    
    /* add a text data */
    if (packet->graphic_type == UI_GRAPH_TEXT) {
      
      /* fail if no text data provided */
      if (!buf) return 0;
      
      /* close open multi-graph packet at the tail */
      sealBuffer(i);
      /* switch if no enough space */
      if (gbuf_tail[i] + 6 + 45 >= GRAPHIC_CMD_BUFFER_SIZE) {
        /* TODO: unseal the buffer */
        gbuf_lock[i] = 0;
        continue;
      }
      /* create new header */
      uint8_t* tail = gbuf[i] + gbuf_tail[i];
      initHeader(tail, DCMD_UI_TEXT);
      /* copy packet and buf in */
      memcpy(tail+6, packet, 15);
      strncpy((char*)tail+6+15, buf, 30);
      /* update the tail */
      gbuf_tail[i] += 6 + 45;
      
    /* add a plain graph data */
    } else {
      
      /* if there are already 7 graphs then close the packet */
      if (gbuf_count[i] == 7) sealBuffer(i);
      
      /* check for space */
      /* if no open multi-graph packet then create one */
      if (gbuf_count[i] == 0) {
        /* switch if no enough space */
        if (gbuf_tail[i] + 6 + 15 >= GRAPHIC_CMD_BUFFER_SIZE) {
          gbuf_lock[i] = 0;
          continue;
        }
        /* create new header */
        initHeader(gbuf[i]+gbuf_tail[i], DCMD_UI_INVALID);
        gbuf_tail[i] += 6;
      } else { /* append data to open packet */
        /* packet number after appending and the difference */
        int n = ceiling(gbuf_count[i]+1), m = n - gbuf_count[i];
        /* switch if no enough space */
        if (gbuf_tail[i] + 15*m >= GRAPHIC_CMD_BUFFER_SIZE) {
          gbuf_lock[i] = 0;
          continue;
        }
      }
      /* copy packet in */
      memcpy(gbuf[i] + gbuf_tail[i], packet, 15);
      /* update buffer tail */
      gbuf_tail[i] += 15;
      /* update packet counter */
      ++gbuf_count[i];
      
    }
    gbuf_lock[i] = 0;
    return 1;
  }
  /* no buffer usable and operation failed */
  return 0;
}

int clearGraphic(enum UI_CLEAR_OP_TYPE op, uint8_t layer) {
  /* try all buffers */
  for (int i = 0; i < 2; ++i) {
    /* switch if this buffer is in use */
    if (gbuf_lock[i]) continue;
    /* lock the buffer */
    gbuf_lock[i] = 1;
    
    /* close tailing open multi-graph packet */
    sealBuffer(i);
    /* switch if no enough space */
    if (gbuf_tail[i] + 6 + 2 >= GRAPHIC_CMD_BUFFER_SIZE) {
      gbuf_lock[i] = 0;
      continue;
    }
    /* create new header */
    initHeader(gbuf[i] + gbuf_tail[i], DCMD_UI_CLEAR);
    /* set packet data */
    gbuf[i][gbuf_tail[i]+6] = op;
    gbuf[i][gbuf_tail[i]+7] = layer;
    /* update the tail */
    gbuf_tail[i] += 6+2;
    
    gbuf_lock[i] = 0;
    return 1;
  }
  /* no buffer usable and operation failed */
  return 0;
}

int8_t txid = -1;
uint16_t txp;

/* require the buffer be locked prehead */
int graphTxInit(int bufID) {
  /* do nothing if there is a transmission */
  if (txid != -1) return 0;
  /* close tailing open multi-graph packet */
  sealBuffer(bufID);
  txid = bufID;
  txp = 0;
  return 1;
}

int graphTxFinish() {
  if (txid == -1) return 0;
  /* reset buffer */
  gbuf_tail[txid] = 0;
  gbuf_count[txid] = 0;
  /* unlock the buffer */
  gbuf_lock[txid] = 0;
  txid = -1;
  return 1;
}

int graphTxNext(void** pBuf, int* pSize, int* pCmd) {
  /* fail if not init or reach end of buffer */
  if (txid == -1 || txp >= gbuf_tail[txid]) return 0;
  
  /* set pBuf to head of next packet */
  *pBuf = gbuf[txid] + txp;
  /* read the lower byte of operation type */
  switch (gbuf[txid][txp]) {
    case DCMD_UI_CLEAR & 0xff:
      *pSize = 6+2;
      break;
    case DCMD_UI_OP_1 & 0xff:
      *pSize = 6+15*1;
      break;
    case DCMD_UI_OP_2 & 0xff:
      *pSize = 6+15*2;
      break;
    case DCMD_UI_OP_5 & 0xff:
      *pSize = 6+15*5;
      break;
    case DCMD_UI_OP_7 & 0xff:
      *pSize = 6+15*7;
      break;
    case DCMD_UI_TEXT & 0xff:
      *pSize = 6+45;
      break;
    default: /* impossible dcmd */
      *pSize = 0;
  }
  if (!*pSize) { /* unrecoverable */
    /* give up all following data */
    return 0;
  }
  txp += *pSize;
  *pCmd = CMD_EXT_STUDENT_DATA;
  return 1;
}

int gc=0, sc=0, fc=0, bws=0, aws=0;

static void sender(void * _) {
  uint8_t* buf;
  int size, cmd;
  while (1) {
    /* if a buffer is transmitting */
    if (txid != -1) {
      if (graphTxNext((void**)&buf, &size, &cmd)) {
        comm_transmit_data(uart_reference.huart, GRAPHIC_SOF, cmd, buf, size);
        ++gc;
        #if TX_IT_SIGNAL_DISABLED
          /* 128 bytes @115200 baud/s takes at least 1.1ms */
          delay(2);
        #else
          ++bws;
				#warning need testing
          osThreadFlagsWait(0x0001,osFlagsWaitAny,2);
          ++aws;
        #endif
      } else {
        ++fc;
        graphTxFinish();
      }
    } else { /* start a new transmission */
      ++sc;
      for (int i = 0; i < 2; ++i) {
        if (gbuf_lock[i]) continue;
        gbuf_lock[i] = 1;
        /* if there are pending packets then init tx */
        if (gbuf_tail[i]) {
          graphTxInit(i);
          break;
        } else { /* unlock the empty buffer */
          gbuf_lock[i] = 0;
        }
      }
      /* sleep a while if not able to send any data */
      if (txid == -1) delay(5);
    }
  }
}

void graphSenderInit() {
  senderID = osThreadCreate("graphSender",sender, NULL, osPriorityNormal, 128);
}

extern UART_HandleTypeDef huart6;

uint8_t ic1, ic2, ic3;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  ++ic1;
  /* do nothing if not about graphic data */
  if (huart != &huart6) return;
  if (txid == -1) return;
  ++ic2;
  #if ! TX_IT_SIGNAL_DISABLED
	  osThreadFlagsSet(senderID,0x0001);
    ++ic3;
  #endif
}

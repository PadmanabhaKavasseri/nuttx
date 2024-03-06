/****************************************************************************
 *
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *
 ****************************************************************************/
#include "qrc.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/
struct qrc_s
{
  pthread_mutex_t pipe_list_mutex;
  pthread_mutex_t qrc_write_mutex;
  qrc_pipe_s pipe_list[64];
  int fd;
  TinyFrame *tf;
  qrc_thread_pool msg_threadpool;
  qrc_thread_pool control_threadpool;
  uint8_t pipe_cnt;
  bool peer_pipe_list_ready;
};

static struct qrc_s g_qrc;

#define QRC_MSG_TIME_OUT_MS (500)  /* ms */
#define QRC_MSG_TIME_OUT_S (5)  /* s */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define QRC_THREAD_NUM (1)
#define MCB_RESET_MAGIC_CMD 0x7102
#define DEFAULT_TF_MSG_TYPE 0x22

#ifdef QRC_RB5
#define QRC_IOC_MAGIC 'q'
#define QRC_FIONREAD _IO(QRC_IOC_MAGIC, 5)
#define QRC_RESET_MCB _IO(QRC_IOC_MAGIC, 2)
#define QRC_FD ("/dev/qrc")
static void sig_handler(int sig)
{
  close(g_qrc.fd);
  printf("actually close..........\n");
}
#define QRC_BOOT_APP  '2'
#endif

#ifdef QRC_MCB
#define QRC_FD ("/dev/ttyS2")
#define QRC_FIONREAD FIONREAD
#endif

#define QRC_HW_SYNC_MSG "OK"

/****************************************************************************
 * Private Functions
 ****************************************************************************/
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len);
TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg);
void *read_response(void *args);
void qrc_control_pipe_callback(qrc_pipe_s *pipe, void * data, size_t len, bool response);
void end_timeout(const uint8_t pipe_id);
static void qrc_msg_cb_work(struct qrc_msg_cb_args_s args);
static int qrc_hardware_sync(int qrc_fd);

/****************************************************************************
 * @intro: send TF frame
 * @param tf: tf
 * @param buff: TF frame
 * @param len: length of buff
 ****************************************************************************/
void TF_WriteImpl(TinyFrame *tf, const uint8_t *buff, uint32_t len)
{
  uint32_t write_cnt = 0;
  write_cnt = write(g_qrc.fd, buff, len);
  if (write_cnt != len)
  {
    printf("ERROR: Write failed!\n");
  }
}

/****************************************************************************
 * @intro: for connection establishment of new pipe, can be only used by pipe_list[0]
 * @param pipe_name: pipe name of caller
 * @param pipe_id: pipe id of caller
 * @param cmd: enum qrc_msg_cmd
 * @return: result of TF_Send()
 ****************************************************************************/
bool qrc_write_request(const char *pipe_name, const uint8_t pipe_id, const enum qrc_msg_cmd cmd)
{
  qrc_frame qrcf;
  qrcf.receiver_id = 0;
  qrcf.ack = NO_ACK;

  qrc_msg msg;
  msg.cmd = cmd;
  msg.pipe_id = pipe_id;
  memset(msg.pipe_name, '\0', 10);
  memcpy(msg.pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));

  bool send_result = qrc_frame_send(&qrcf, (uint8_t*)(&msg), sizeof(qrc_msg), true);
  if(true == send_result && (QRC_REQUEST== cmd || QRC_WRITE_LOCK == cmd || QRC_WRITE_UNLOCK == cmd))
  {
    g_qrc.pipe_list[0].timeout_happen = false;
    start_timeout(0);
  }
  return send_result;
}

/****************************************************************************
 * @intro: this function will be called when tf receive msg
 * @param tf: receiver
 * @param msg: msg received
 * @return: TF_STAY
 ****************************************************************************/
TF_Result read_response_listener(TinyFrame *tf, TF_Msg *msg)
{
  qrc_frame qrcf;
  memcpy(&qrcf, msg->data, sizeof(qrc_frame));

  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(qrcf.receiver_id);
  if(NULL == p)
  {
    printf("ERROR: here is no pipe with peer pipe id %u, receive failed!\n", qrcf.receiver_id);
  }
  else
  {
    if(ACK == qrcf.ack)
    {
      qrc_write_request(p->pipe_name, p->peer_pipe_id, QRC_ACK);
    }
    if(NULL != p->cb)
    {
      struct qrc_msg_cb_args_s args;
      args.fun_cb = p->cb;
      args.pipe = p;
      args.len = msg->len - sizeof(qrc_frame);
      args.data = (uint8_t*)malloc(args.len);
      memcpy(args.data, msg->data +sizeof(qrc_frame), args.len);
      args.response = false;
      qrc_threadpool_add_work(g_qrc.msg_threadpool, qrc_msg_cb_work, args);
    }
  }

  return TF_STAY;
}

/****************************************************************************
 * @intro: callback function of pipelist[0], whose pipe id is 0
 * @param pipe: pipelist[0]
 * @param data: data received
 * @param len: length of data
 * @param response: no use
 ****************************************************************************/
void qrc_control_pipe_callback(qrc_pipe_s *pipe, void * data, size_t len, bool response)
{
  qrc_msg qmsg;
  memcpy(&qmsg, data, sizeof(qrc_msg));
  uint8_t cmd = qmsg.cmd;
  uint8_t pipe_id = qmsg.pipe_id; /*pipe id of receiver*/
  char pipe_name[10] = "\0";
  memcpy(pipe_name, qmsg.pipe_name, 10);

  switch (cmd)
    {
      case QRC_REQUEST:
        {
          qrc_pipe_s *p = qrc_pipe_insert(pipe_name);
          if(p == NULL)
            {
            printf("ERROR: corresponding pipe(%s) create failed!\n", pipe_name);
            }
          else
            {
              printf("DEBUG: qrc_control_pipe_callback get QRC_REQUEST peer_id =%d\n",pipe_id);
              p->peer_pipe_id = pipe_id;
              qrc_write_request(p->pipe_name, p->pipe_id, QRC_RESPONSE);
            }
          break;
        }
      case QRC_RESPONSE:
        {
          qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
          if(p == NULL)
            {
              printf("ERROR: pipe name(%s) doesn't exit, can not handle QRC_RESPONSE!\n", pipe_name);
            }
          else
            {
              printf("DEBUG: qrc_control_pipe_callback get QRC_RESPONSE peer_id =%d\n",pipe_id);
              p->peer_pipe_id = pipe_id;
              end_timeout(QRC_CONTROL_PIPE_ID);
            }
          break;
        }
      case QRC_WRITE_LOCK:
        {
          qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
          printf("DEBUG: qrc_control_pipe_callback send QRC_WRITE_LOCK pipe_id =%d\n",p->pipe_id);
          qrc_write_request(p->pipe_name, p->pipe_id, QRC_WRITE_LOCK_ACK);
          qrc_bus_lock();
          break;
        }
      case QRC_WRITE_UNLOCK:
        {
          qrc_pipe_s *p = qrc_pipe_find_by_name(pipe_name);
          printf("DEBUG: qrc_control_pipe_callback send QRC_WRITE_UNLOCK pipe_id =%d\n",p->pipe_id);
          qrc_write_request(p->pipe_name, p->pipe_id, QRC_WRITE_UNLOCK_ACK);
          qrc_bus_unlock();
          break;
        }
      case QRC_ACK:
        {
          qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
          printf("DEBUG: qrc_control_pipe_callback get QRC_ACK pipe_id =%d\n",p->pipe_id);
          end_timeout(p->pipe_id); /*pipe_id == user id*/
          break;
        }
      case QRC_WRITE_LOCK_ACK:
      case QRC_WRITE_UNLOCK_ACK:
        {
          printf("DEBUG: qrc_control_pipe_callback get  QRC_WRITE_LOCK_ACK or QRC_WRITE_UNLOCK_ACK  peer_pipe_id =%d\n",pipe_id);
          end_timeout(QRC_CONTROL_PIPE_ID);
          break;
        }
      case QRC_CONNECT_REQUEST:
        {
          g_qrc.peer_pipe_list_ready = true;
          printf("DEBUG: qrc_control_pipe_callback get QRC_CONNECT_REQUEST\n");
          qrc_write_request("", QRC_CONTROL_PIPE_ID, QRC_CONNECT_RESPONSE);
          end_timeout(QRC_CONTROL_PIPE_ID);
          break;
        }
      case QRC_CONNECT_RESPONSE:
        {
          g_qrc.peer_pipe_list_ready = true;
          printf("DEBUG: qrc_control_pipe_callback get QRC_CONNECT_RESPONSE\n");
          end_timeout(QRC_CONTROL_PIPE_ID);
          break;
        }
      default :
        printf("WARNING: qrc_control_pipe_callback cmd=%d is invalid\n", cmd);
        break;
    }
}

/****************************************************************************
 * @intro: initilize a new pipe
 * @return: new pipe
 ****************************************************************************/
qrc_pipe_s qrc_pipe_node_init(void)
{
  qrc_pipe_s node;
  memset(node.pipe_name, '\0', 10);

  if(0 != pthread_cond_init(&node.pipe_cond, NULL))
  {
    printf("\nERROR: pipe cond initalize failed!\n");
    return node;
  }
  if(0 != pthread_mutex_init(&node.pipe_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    return node;
  }
  node.pipe_id = 255;
  node.peer_pipe_id = 255;
  node.timeout_happen = false;
  node.cb = NULL;
  return node;
}

/****************************************************************************
 * @intro: initilize the pipe list
 ****************************************************************************/
bool qrc_pipe_list_init(void)
{
  pthread_mutex_lock(&g_qrc.pipe_list_mutex);

  g_qrc.pipe_list[QRC_CONTROL_PIPE_ID] = qrc_pipe_node_init();
  g_qrc.pipe_list[QRC_CONTROL_PIPE_ID].pipe_id = QRC_CONTROL_PIPE_ID;
  g_qrc.pipe_list[QRC_CONTROL_PIPE_ID].peer_pipe_id = QRC_CONTROL_PIPE_ID;
  char *pipe_name = "QRC_CTL";

  memcpy(g_qrc.pipe_list[QRC_CONTROL_PIPE_ID].pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
  g_qrc.pipe_list[QRC_CONTROL_PIPE_ID].cb = qrc_control_pipe_callback;
  g_qrc.pipe_cnt = 1;

  pthread_mutex_unlock(&g_qrc.pipe_list_mutex);
  qrc_write_request("", QRC_CONTROL_PIPE_ID, QRC_CONNECT_REQUEST);
  return (start_timeout(QRC_CONTROL_PIPE_ID) == false)? true:false;
}

/****************************************************************************
 * @intro: create a new pipe named pipe_name
 * @return: pointer of new pipe or exited pipe
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_insert(const char *pipe_name)
{
  pthread_mutex_lock(&g_qrc.pipe_list_mutex);
  qrc_pipe_s *lt = g_qrc.pipe_list;
  if(g_qrc.pipe_cnt >= 63)
  {
    pthread_mutex_unlock(&g_qrc.pipe_list_mutex);
    return NULL;
  }
  qrc_pipe_s *find_res = qrc_pipe_find_by_name(pipe_name);
  if(NULL == find_res)
  {
    uint8_t new_pipe_index = g_qrc.pipe_cnt;
    g_qrc.pipe_cnt = (g_qrc.pipe_cnt + 1) % 64;
    lt[new_pipe_index] = qrc_pipe_node_init();
    lt[new_pipe_index].pipe_id = new_pipe_index;
    //debug
    lt[new_pipe_index].peer_pipe_id = new_pipe_index;
    memcpy(lt[new_pipe_index].pipe_name, pipe_name, strlen(pipe_name) * sizeof(char));
    find_res = &lt[new_pipe_index];
  }
  pthread_mutex_unlock(&g_qrc.pipe_list_mutex);
  return find_res;
}

/****************************************************************************
 * @intro: find a pipe named pipe_name
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_find_by_name(const char *pipe_name)
{
  for(uint8_t i = 1; i < g_qrc.pipe_cnt; i++)
  {
    if(0 == strcmp(g_qrc.pipe_list[i].pipe_name, pipe_name))
    {
      return &g_qrc.pipe_list[i];
    }
  }
  return NULL;
}

/****************************************************************************
 * @intro: find a pipe by its pipe_id
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_find_by_pipeid(const uint8_t pipe_id)
{
  if(g_qrc.pipe_cnt <= pipe_id)
  {
    return NULL;
  }
  return &g_qrc.pipe_list[pipe_id];
}

/****************************************************************************
 * @intro: modify the data of pipe
 * @return: pointer of pipe or NULL
 ****************************************************************************/
qrc_pipe_s *qrc_pipe_modify_by_name(const char *pipe_name, const qrc_pipe_s *new_data)
{
  return NULL;
}

/****************************************************************************
 * @intro: send TF frame
 * @param qrcf: qrcf_frame(sync_mode + ack + receiver_id)
 * @param data: qrc_msg(qrc_msg_cmd + pipe id + pipe name) or user data
 * @param len: length of data
 * @param qrc_write_lock: whether hold lock to ensure the integrity of the frame, default is true
 * @return: result of TF_Send()
 ****************************************************************************/
bool qrc_frame_send(const qrc_frame *qrcf, const uint8_t *data, const size_t len, const bool qrc_write_lock)
{
  if(true == qrc_write_lock)
  {
    pthread_mutex_lock(&g_qrc.qrc_write_mutex);
  }
  TF_Msg msg;
  TF_ClearMsg(&msg);
  msg.type = DEFAULT_TF_MSG_TYPE;
  uint8_t *msg_qrc = (uint8_t*)malloc(sizeof(qrc_frame) + len);


  memcpy(msg_qrc, qrcf, sizeof(qrc_frame));
  memcpy(msg_qrc + sizeof(qrc_frame), data, len);
  msg.data = msg_qrc;
  msg.len = sizeof(qrc_frame) + len;

//printf("DEBUG qrc_frame_send data[0]=%d,data[1]=%d,data[5]=%d\n",*(int*)(msg_qrc+1),*(int*)(msg_qrc+5),*(int*)(msg_qrc+9));
  bool send_res = TF_Send(g_qrc.tf, &msg);
  free(msg_qrc);
  if(true == qrc_write_lock)
  {
    pthread_mutex_unlock(&g_qrc.qrc_write_mutex);
  }

  return send_res;
}

/****************************************************************************
 * @intro: start the timeout of pipe whose pipe id is pipe_id
 * @param pipe_id: pipe id
 ****************************************************************************/
bool start_timeout(const uint8_t pipe_id)
{
  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
  if (p ==NULL)
    {
      printf("ERROR: start_timeout input pipe id is invalid\n");
    }

  p->timeout_happen = false;
  struct timeval now;
  gettimeofday(&now, NULL);

  struct timespec outtime;
  outtime.tv_sec = now.tv_sec + QRC_MSG_TIME_OUT_S;
  outtime.tv_nsec = now.tv_usec; /*500ms*/

  pthread_mutex_lock(&p->pipe_mutex);
  if(0 != pthread_cond_timedwait(&p->pipe_cond, &p->pipe_mutex, &outtime))
  {
    printf("\nERROR: pipe(%s) TIMEOUT!\n", p->pipe_name);
    p->timeout_happen = true;
  }
  pthread_mutex_unlock(&p->pipe_mutex);
  return p->timeout_happen;
}

/****************************************************************************
 * @intro: wake up the timeout of pipe whose pipe id is pipe_id
 * @param pipe_id: pipe id
 ****************************************************************************/
void end_timeout(const uint8_t pipe_id)
{
  qrc_pipe_s *p = qrc_pipe_find_by_pipeid(pipe_id);
  if (p ==NULL)
    {
      printf("ERROR: start_timeout input pipe id is invalid\n");
    }

  pthread_mutex_lock(&p->pipe_mutex);
  if(0 != pthread_cond_signal(&p->pipe_cond))
  {
    printf("\nERROR: Can not wake up main thread!\n");
    pthread_mutex_unlock(&p->pipe_mutex);
    return;
  }
  pthread_mutex_unlock(&p->pipe_mutex);
}

/****************************************************************************
 * @intro: lock the qrc_write_mutex
 ****************************************************************************/
void qrc_bus_lock(void)
{
  pthread_mutex_lock(&g_qrc.qrc_write_mutex);
}

/****************************************************************************
 * @intro: unlock the qrc_write_mutex
 ****************************************************************************/
void qrc_bus_unlock(void)
{
  pthread_mutex_unlock(&g_qrc.qrc_write_mutex);
}

/****************************************************************************
 * @intro: query if pipe0's timeout happen
 ****************************************************************************/
bool qrc_cmd_timeout(void)
{
  return g_qrc.pipe_list[0].timeout_happen;
}

/****************************************************************************
 * @intro: thread of reading response
 ****************************************************************************/
void *read_response(void *args)
{
  int readable_len;
  while(1)
  {
    readable_len = 0;
    if(ioctl(g_qrc.fd, QRC_FIONREAD, &readable_len) < 0)
    {
      printf("\nERROR: qrc get readable size fail!\n");
      return NULL;
    }
    if(readable_len > 0)
    {
      uint8_t *buf = malloc(readable_len * sizeof(uint8_t));
      int read_len = read(g_qrc.fd, buf, readable_len);
      if(read_len > 0) {
        TF_Accept(g_qrc.tf, (uint8_t*)buf, (uint32_t)read_len);
      }
      free(buf);
    }
    else
      usleep(100);
  }
}

/****************************************************************************
 * @intro: execute the function args.fun_cb
 * @param args: thread holder
 ****************************************************************************/
static void qrc_msg_cb_work(struct qrc_msg_cb_args_s args)
{
  args.fun_cb(args.pipe, args.data, args.len, args.response);
  free(args.data);
}

uint8_t get_pipe_number(void)
{
  return g_qrc.pipe_cnt;
}

/* Hardware sync */
static int qrc_hardware_sync(int qrc_fd)
{
  int readable_len = 0;
  int try = 10;
  char ack[] =  QRC_HW_SYNC_MSG;
  uint32_t write_cnt = 0;

#ifdef QRC_RB5
  /* reset MCB */
  ioctl(qrc_fd, QRC_RESET_MCB);
  sleep(2);

  /* Boot APP */
  char mcb_boot_app = QRC_BOOT_APP;

  write_cnt = write(qrc_fd, &mcb_boot_app, 1);
  if (write_cnt != 1)
  {
    printf("ERROR: qrc bus write failed!\n");
  }

  printf("DEBUG: qrc start bus sync \n");
  while(try > 0)
    {
      try --;
      sleep(1);
      if(ioctl(qrc_fd, QRC_FIONREAD, &readable_len) < 0)
        {
          printf("\nERROR: qrc get readable size fail!\n");
          close(qrc_fd);
          return -1;
        }

      if(readable_len >= 3)
      {
        char *buf = malloc(readable_len * sizeof(char));
        int read_len = read(qrc_fd, buf, readable_len);
        if(read_len >= 3)
          {
            int count = 0;
            while(count <= (read_len -2))
            {
              if (buf[count] == 'O' && buf[count+1] =='K')
                {
                  write_cnt = write(qrc_fd, ack, sizeof(ack));
                  if (write_cnt != sizeof(ack))
                    {
                      printf("ERROR: qrc bus write SYNC MSG failed!\n");
                      close(qrc_fd);
                      return -1;
                    }
                  printf("DEBUG: qrc bus SYNC done\n");
                  return 0;
                }
              count = count +1;
            }
          }
        free(buf);
      }
      printf("DEBUG: qrc bus write SYNC try = %d \n",try);
    }

#else // QRC_MCB
  while(try > 0)
    {
      write_cnt = write(qrc_fd, ack, sizeof(ack));
      if (write_cnt != sizeof(ack))
        {
          printf("ERROR: qrc bus write SYNC MSG failed!\n");
          close(qrc_fd);
          return -1;
        }

      /* check if received ACK msg */
      if(ioctl(qrc_fd, QRC_FIONREAD, &readable_len) < 0)
        {
          printf("\nERROR: qrc get readable size fail!\n");
          close(qrc_fd);
          return -1;
        }

      if(readable_len >= 3)
        {
          char *buf = malloc(readable_len * sizeof(char));
          int read_len = read(qrc_fd, buf, readable_len);
          if(read_len >= 3)
            {
              int count = 0;
              while(count <= (read_len -2))
                {
                  if (buf[count] == 'O' && buf[count+1] =='K')
                    {
                      printf("DEBUG: qrc bus sync done\n");
                      return 0;
                    }
                  count = count +1;
                }
            }
          free(buf);
        }
      printf("DEBUG: qrc bus write SYNC try = %d \n",try);
      try --;
      sleep(1);
      
    }

#endif
  printf("\nERROR: qrc BUS try to sync failed \n");
  return -1;
}

/****************************************************************************
 * Public function
 ****************************************************************************/

/****************************************************************************
 * @intro: initilial
 * @return: result of initilial
 ****************************************************************************/
bool qrc_init(void)
{
  g_qrc.fd = open(QRC_FD, O_RDWR);
  if(-1 == g_qrc.fd)
  {
    printf("ERROR: %s open failed!\n", QRC_FD);
    close(g_qrc.fd);
    return false;
  }

  if(0 != qrc_hardware_sync(g_qrc.fd))
    {
      printf("ERROR: qrc HW sync failed!\n");
      close(g_qrc.fd);
      return false;
    }

  if(0 != pthread_mutex_init(&g_qrc.pipe_list_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    close(g_qrc.fd);
    return false;
  }
  if(0 != pthread_mutex_init(&g_qrc.qrc_write_mutex, NULL))
  {
    printf("\nERROR: pipe mutex initalize failed!\n");
    close(g_qrc.fd);
    return false;
  }
  g_qrc.peer_pipe_list_ready = false;
  g_qrc.msg_threadpool = qrc_thread_pool_init(QRC_THREAD_NUM);
  g_qrc.tf = TF_Init(TF_MASTER);
  TF_AddGenericListener(g_qrc.tf, read_response_listener);

  #ifdef QRC_RB5
  signal(SIGINT, sig_handler);
  #endif
  pthread_t t;
  pthread_create(&t, NULL, read_response, NULL);

  return qrc_pipe_list_init();
}

void qrc_pipe_threads_join(void)
{
  qrc_threads_join(g_qrc.msg_threadpool);
}
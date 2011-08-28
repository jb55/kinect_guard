
#include <sys/time.h>
#include <sys/types.h>
#include <signal.h>
#include <zmq.h>
#include <libfreenect.h>
#include <assert.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// constants
static int THRESHOLD = 10;
static int CUTOFF = 50;
static const int resolution = FREENECT_RESOLUTION_MEDIUM;

// globals
volatile sig_atomic_t running = 1;
uint32_t last_timestamp = 0;
FILE *rgb_stream = 0;
void *zmq_sock = 0;
void *zmq_cmd_sock = 0;
int _is_recording = 0;

void *last_frame = 0;
int last_brightness = 0;
int last_frame_size = 0;
int last_client_heartbeat = 0;
int current_format = FREENECT_VIDEO_RGB;
int requested_format = FREENECT_VIDEO_RGB;

// function prototypes
static FILE *open_ffmpeg(char *output_filename, int w, int h);
static int check_last_frame(int32_t size);
static void rgb_cb(freenect_device *dev, void *data, uint32_t timestamp);
static void init(int use_ffmpeg);
static void signal_cleanup(int num);
static void publish_msg(void *socket, const char* str);
static int is_recording();
static void movement_event(void* socket, int delta);
static void brightness_event(void* socket, int new_brightness);
static void record_frame(void *data, int size, FILE *stream);
static void switch_to_ir();
static void switch_to_rgb();

// msgs
enum command_types {
    CMD_SWITCH_TO_IR
  , CMD_SWITCH_TO_RGB
  , CMD_SET_CUTOFF
  , CMD_GET_CUTOFF
  , CMD_GET_VIDEO_MODE
  , CMD_NOPARSE
};

// zeroMQ
void publish_msg(void *socket, const char* str) {
  char* data;
  size_t size = strlen(str);

  zmq_msg_t msg;
  zmq_msg_init_size(&msg, size);
  data = (char*)zmq_msg_data(&msg);

  memcpy(data, str, size);

  zmq_send(socket, &msg, ZMQ_NOBLOCK);
}

int parse_msg_type(char* data, char** rest) {
  char* space_pos;
  int type, cmd_size;
  space_pos = data;
  while (*space_pos++ != ' ' && space_pos != '\0');
  space_pos--;

  *space_pos = '\0';

  if (strcmp(data, "switch_to_ir") == 0)
    type = CMD_SWITCH_TO_IR;
  else if (strcmp(data, "switch_to_rgb") == 0)
    type = CMD_SWITCH_TO_RGB;
  else if (strcmp(data, "set_cutoff") == 0)
    type = CMD_SET_CUTOFF;
  else if (strcmp(data, "get_cutoff") == 0)
    type = CMD_GET_CUTOFF;
  else if (strcmp(data, "get_video_mode") == 0)
    type = CMD_GET_VIDEO_MODE;

  *space_pos = ' ';
  *rest = space_pos + 1;

  return type;
}

void handle_msg(zmq_msg_t *msg) {
  char tmp[128];
  char *rest, *data = (char*)zmq_msg_data(msg);
  char *mode;
  int size = zmq_msg_size(msg);

  memcpy(tmp, data, size);
  tmp[size+1] = '\0';

  int type = parse_msg_type(tmp, &rest);
  int ok = 0;
  int needs_confirm = 1;
  int cutoff;

  switch (type) {
    case CMD_SWITCH_TO_IR:
      switch_to_ir();
      ok = 1;
      publish_msg(zmq_cmd_sock, tmp);
      break;
    case CMD_SWITCH_TO_RGB:
      switch_to_rgb();
      ok = 1;
      publish_msg(zmq_cmd_sock, tmp);
      break;
    case CMD_SET_CUTOFF:
      sscanf(rest, "%d", &cutoff);
      CUTOFF = cutoff;
      publish_msg(zmq_cmd_sock, tmp);
      break;
    default:
    case CMD_GET_CUTOFF:
      sprintf(tmp, "get_cutoff %d", CUTOFF);
      publish_msg(zmq_cmd_sock, tmp);
      ok = 1;
      break;
    case CMD_GET_VIDEO_MODE:
      mode = current_format == FREENECT_VIDEO_RGB? "rgb" : "ir";
      sprintf(tmp, "get_video_mode %s", mode);
      publish_msg(zmq_cmd_sock, tmp);
      ok = 1;
      break;
    case CMD_NOPARSE:
      break;
  }

  if (!ok)
    publish_msg(zmq_cmd_sock, "invalid_cmd");
}


void signal_cleanup(int num) {
	running = 0;
	printf("Caught signal, cleaning up\n");
	signal(SIGINT, signal_cleanup);
}


// initialize our last_frame buffer or resize it if the
// video mode has changed
int check_last_frame(int32_t size)
{

  if (!last_frame) {
    last_frame = malloc(size);
    last_frame_size = size;
    return 1;
  } else if (last_frame_size != size) {
    last_frame = realloc(last_frame, size);
    last_frame_size = size;
    return 1;
  }

  return 0;
}

void brightness_event(void *socket, int new_brightness)
{
  char data[64];
  sprintf(data, "brightness %d", new_brightness);
  publish_msg(socket, data);
}

void movement_event(void *socket, int delta)
{
  char data[64];
  sprintf(data, "movement %d", delta);
  publish_msg(socket, data);
}

void switch_to_rgb() {
  requested_format = FREENECT_VIDEO_RGB;
  CUTOFF = 50;
}

void change_video_format(freenect_device *dev, int format) {
  freenect_stop_video(dev);
  freenect_set_video_mode(dev, freenect_find_video_mode(resolution, format));
  freenect_start_video(dev);
  current_format = format;
}

void switch_to_ir() {
  requested_format = FREENECT_VIDEO_IR_8BIT;
  CUTOFF = 80;
}

void rgb_cb(freenect_device *dev, void *data, uint32_t timestamp)
{
  int first = 0;
  int delta = 0;
  int diff = 0;
  int brightness = 0, average_brightness = 0;
  unsigned char *new_data, *old_data;
  int size = freenect_get_current_video_mode(dev).bytes;

  first = check_last_frame(size);

  new_data = (char*)data;
  old_data = (char*)last_frame;

  // check difference
  int i;
  unsigned char r, g, b;
  unsigned char or, og, ob;
  for (i = 0; i < size; i += 3) {
    r  = *new_data++;  g = *new_data++;  b = *new_data++;
    or = *old_data++; og = *old_data++; ob = *old_data++;

    // average the distance in each channel
    diff = (abs(r - or) + abs(g - og) + abs(b - ob)) / 3;
    brightness += (r + g + b) / 3;

    if (diff > CUTOFF)
      delta += diff - CUTOFF;
  }

  average_brightness = brightness / (size / 3);

  // notify why brightness changes
  if (average_brightness != last_brightness) {
    last_brightness = average_brightness;
    brightness_event(zmq_sock, average_brightness);
  }

  /*
  if (current_format == FREENECT_VIDEO_RGB && average_brightness <= 10)
    switch_to_ir();
  else if (current_format == FREENECT_VIDEO_IR_8BIT && average_brightness >= 50)
    switch_to_rgb();

  printf("avg brightness: %d, delta: %d\n", average_brightness, delta);
  */

  if (delta >= THRESHOLD) {
//  printf("frame delta: %d, (%d)\n", delta, diff);
    movement_event(zmq_sock, delta);
  }

  // copt current frame into last_frame for next callback
  memcpy(last_frame, data, size);

  // pipe data to ffmpeg
  if (is_recording())
    record_frame(data, size, rgb_stream);

}

static int is_recording() {
  return _is_recording;
}

static void record_frame(void *data, int size, FILE *stream) {
  fwrite(data, size, 1, stream);
}

static void init(int use_ffmpeg) {
	freenect_context *ctx;
	freenect_device *dev;
  void *zmq_ctx;

  zmq_ctx = zmq_init(1);

  zmq_sock = zmq_socket(zmq_ctx, ZMQ_PUB);
  zmq_cmd_sock = zmq_socket(zmq_ctx, ZMQ_REP);

  zmq_bind(zmq_sock, "tcp://127.0.0.1:5555");
  zmq_bind(zmq_cmd_sock, "tcp://127.0.0.1:5556");

	if (freenect_init(&ctx, 0)) {
		printf("Error: Cannot get context\n");
		return;
	}

	freenect_select_subdevices(ctx, (freenect_device_flags)
    ( FREENECT_DEVICE_MOTOR
    | FREENECT_DEVICE_CAMERA
//  | FREENECT_DEVICE_AUDIO
    )
  );

	if (freenect_open_device(ctx, &dev, 0)) {
		printf("Error: Cannot get device\n");
		return;
	}

	freenect_set_video_mode(dev, freenect_find_video_mode(resolution,
                                                        FREENECT_VIDEO_RGB));
	freenect_start_video(dev);
  freenect_set_video_callback(dev, rgb_cb);

	while (running && freenect_process_events(ctx) >= 0) {
    zmq_msg_t msg;
    zmq_msg_init(&msg);

    while (zmq_recv(zmq_cmd_sock, &msg, ZMQ_NOBLOCK) == 0) {
      handle_msg(&msg);
    }

    if (requested_format != current_format) {
      change_video_format(dev, requested_format);
    }
  }

  zmq_close(zmq_sock);
  zmq_close(zmq_cmd_sock);

  zmq_term(zmq_ctx);

	freenect_stop_video(dev);
	freenect_close_device(dev);
	freenect_shutdown(ctx);
}


static FILE* open_ffmpeg(char *output_filename, int w, int h)
{
	char cmd[1024];
  static const char* ffmpeg_opts =
    "-aspect 4:3 -r 20 -vcodec msmpeg4 -b 30000k";

	snprintf(cmd, 1024, "ffmpeg -pix_fmt rgb24 -s %dx%d -f rawvideo "
			 "-i /dev/stdin %s %s", w, h, ffmpeg_opts, output_filename);

	fprintf(stderr, "%s\n", cmd);

	FILE* proc = popen(cmd, "w");
	if (!proc) {
		printf("Error: Cannot run ffmpeg\n");
		exit(1);
	}

	return proc;
}


int main(int argc, const char *argv[]) {

	signal(SIGINT, signal_cleanup);
  init(1 /* use ffmpeg */);

  return 0;
}

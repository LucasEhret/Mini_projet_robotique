#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

uint16_t get_line_position(void);
uint16_t get_line_width(void);
void process_image_start(void);
void start_thread_camera(void);
void pause_thread_camera(void);

#endif /* PROCESS_IMAGE_H */

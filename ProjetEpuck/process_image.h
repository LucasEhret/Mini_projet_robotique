#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
uint16_t get_line_width(void);
void process_image_start(void);
void stop_thread_camera(void);

#endif /* PROCESS_IMAGE_H */

#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H

float get_distance_cm(void);
uint16_t get_line_position(void);
uint16_t get_line_width(void);
void process_image_start(thread_t* thd_pt);
void capture_image_start(thread_t* thd_pt);

#endif /* PROCESS_IMAGE_H */

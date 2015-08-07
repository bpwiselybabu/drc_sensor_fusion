#string[] texts               # text message show on the image

uint16 rows                  # image size
uint16 cols
#uint16[] white_pixel_row     # assuming we are sending a bw image, these are the index for white pixels
#uint16[] white_pixel_col     # assuming we are sending a bw image, these are the index for white pixels

#ContourList contours
uint8[] jpeg

uint16[4] left_foot_row
uint16[4] left_foot_col

uint16[4] right_foot_row
uint16[4] right_foot_col

# just forwarding sf_controller_out
int8 scriptMode
int8 mode
int8 numStepsLeftInScirpt
float32[3] left_foot_rpy
float32[3] right_foot_rpy

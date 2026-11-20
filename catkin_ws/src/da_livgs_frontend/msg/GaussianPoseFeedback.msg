std_msgs/Header header

uint32 source_frame
bool observer_accepted
bool observe_only
uint8 axes_mask
uint8 projection_mode
bool projection_valid
float64 selected_condition_number
bool validation_active
float64[6] validation_ground_truth_update

float64[6] raw_update
float64[6] projected_update
float64[6] bounded_update
float64[36] information

uint8 information_rank
float64 condition_number
float64 gain
float64 maximum_translation_metres
float64 maximum_rotation_degrees
float64 render_coverage
uint32 valid_samples
uint64 mature_gaussians
float64 observer_elapsed_ms

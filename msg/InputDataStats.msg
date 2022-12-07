#  Johns Hopkins University Applied Physics Laboratory
#
# input data statistics message : for thread mode (preprocessing input in callback, rtabmap runs in a thread), statistics at the 
#                                   input preprocessing callback stage. 
#

# transmitting message header, the time when the message was published.
std_msgs/Header header


# over head time elapsed since callback  was last called 
float32 time_elapsed_since_last_called

# preprocessing_elapsed_time: time elapsed for computing the preprocessing section
# total_elapsed_time: total time elapsed of callback (data input processing)

float32 preprocessing_elapsed_time
float32 total_elapsed_time


#include <gst/gst.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

static volatile sig_atomic_t stop = 0;

void handle_sigint(int sig) {
    stop = 1;
}

int main(int argc, char* argv[]) {
    GstElement* pipeline;
    GstBus* bus;
    GstMessage* msg;

    gst_init(&argc, &argv);

    signal(SIGINT, handle_sigint);  // Ctrl+C handler

    pipeline = gst_parse_launch(
        "v4l2src device=/dev/video0 ! "
        "video/x-raw,format=UYVY,width=1920,height=1080 ! "
        "videoconvert ! autovideosink", NULL);

    gst_element_set_state(pipeline, GST_STATE_PLAYING);
    bus = gst_element_get_bus(pipeline);

    // Poll for messages or Ctrl+C
    while (!stop) {
        msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
            GST_MESSAGE_ERROR | GST_MESSAGE_EOS);

        if (msg != NULL) {
            GError* err;
            gchar* debug;

            switch (GST_MESSAGE_TYPE(msg)) {
            case GST_MESSAGE_ERROR:
                gst_message_parse_error(msg, &err, &debug);
                //g_printerr("ERROR: %s\n", err->message);

                gst_message_parse_error(msg, &err, &debug);
                g_printerr("ERROR: %s\n", err->message);
                g_printerr("DEBUG INFO: %s\n", debug);


                g_error_free(err);
                g_free(debug);
                stop = 1;
                break;

            case GST_MESSAGE_EOS:
                g_print("End of stream\n");
                stop = 1;
                break;

            default:
                break;
            }

            gst_message_unref(msg);
        }
    }

    // Clean up
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(pipeline);

    g_print("Exited gracefully.\n");
    return 0;
}

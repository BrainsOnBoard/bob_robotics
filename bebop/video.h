/*
 * This header implements the functionality for reading from the Bebop's video
 * stream.
 *
 * The VideoDecoder class is a tweaked version of the (BSD-licenced) ROS Bebop
 * implementation:
 * https://github.com/AutonomyLab/bebop_autonomy/blob/indigo-devel/bebop_driver/include/bebop_driver/bebop_video_decoder.h
 *
 * The BebopVideoStream class provides a wrapper around VideoDecoder and handles
 * sending commands to the drone via the Bebop class etc.
 */

#pragma once

#ifndef DUMMY_DRONE
// ARSDK includes
extern "C"
{
#include "libARController/ARCONTROLLER_Error.h"
#include "libARController/ARCONTROLLER_Frame.h"
#include "libARSAL/ARSAL_Print.h"
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavformat/avio.h>
#include <libswscale/swscale.h>
}

// These constants no longer seem to be defined on my machine - AD
#ifndef CODEC_CAP_TRUNCATED
#define CODEC_CAP_TRUNCATED AV_CODEC_CAP_TRUNCATED
#define CODEC_FLAG_TRUNCATED AV_CODEC_FLAG_TRUNCATED
#define CODEC_FLAG2_CHUNKS AV_CODEC_FLAG2_CHUNKS
#endif // CODEC_CAP_TRUNCATED

// https://github.com/libav/libav/commit/104e10fb426f903ba9157fdbfe30292d0e4c3d72
// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L697
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(54, 25, 0))
#define AV_CODEC_ID_H264 CODEC_ID_H264
#endif

// https://github.com/libav/libav/blob/33d18982fa03feb061c8f744a4f0a9175c1f63ab/doc/APIchanges#L653
#if (LIBAVCODEC_VERSION_INT < AV_VERSION_INT(51, 42, 0))
#define AV_PIX_FMT_YUV420P PIX_FMT_YUV420P
#endif

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
#define av_frame_alloc avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif
#endif // !DUMMY_DRONE

// C includes
#include <cstdint>

// C++ includes
#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

// POSIX includes
#include <signal.h>

// local includes
#include "bebop.h"

#define VIDEO_FIFO "/tmp/bebop_vid"
#define VIDEO_WIDTH 856
#define VIDEO_HEIGHT 480

namespace BoBRobotics {
namespace Robots {
#ifndef DUMMY_DRONE
class VideoDecoder
{
private:
    static const char *LOG_TAG;

    bool codec_initialized_;
    bool first_iframe_recv_;
    AVFormatContext *format_ctx_ptr_;
    AVCodecContext *codec_ctx_ptr_;
    AVCodec *codec_ptr_;
    AVFrame *frame_ptr_;
    AVFrame *frame_rgb_ptr_;
    AVPacket packet_;
    SwsContext *img_convert_ctx_ptr_;
    AVInputFormat *input_format_ptr_;
    uint8_t *frame_rgb_raw_ptr_;

    bool update_codec_params_;
    std::vector<uint8_t> codec_data_;

    static void ThrowOnCondition(const bool cond, const std::string &message);
    bool InitCodec();
    bool ReallocateBuffers();
    void CleanupBuffers();
    void Reset();

    void ConvertFrameToRGB();

public:
    VideoDecoder();
    ~VideoDecoder();

    bool SetH264Params(uint8_t *sps_buffer_ptr,
                       uint32_t sps_buffer_size,
                       uint8_t *pps_buffer_ptr,
                       uint32_t pps_buffer_size);
    bool Decode(const ARCONTROLLER_Frame_t *bebop_frame_ptr_);
    inline uint32_t GetFrameWidth() const
    {
        return codec_initialized_ ? codec_ctx_ptr_->width : 0;
    }
    inline uint32_t GetFrameHeight() const
    {
        return codec_initialized_ ? codec_ctx_ptr_->height : 0;
    }

    inline const uint8_t *GetFrameRGBRawCstPtr() const
    {
        return frame_rgb_raw_ptr_;
    }
};
#endif // !DUMMY_DRONE

// start BebopVideoStream
using userVideoCallback = void (*)(const uint8_t *frame, void *userdata);

class BebopVideoStream
{
public:
    BebopVideoStream(Bebop *bebop);
    ~BebopVideoStream();
    void startStreaming();
    void startStreaming(userVideoCallback cb, void *userdata);
    void stopStreaming();
    void startMplayer();

private:
    ARCONTROLLER_Device_t *m_Device;
    VideoDecoder *decoder = nullptr;
    FILE *pipe = nullptr;
    userVideoCallback m_UserCallback = nullptr;
    void *m_UserVideoCallbackData;
    pid_t mplayer = 0;

    /*
     * Create and open named pipe for streaming to mplayer.
     */
    inline void openPipe()
    {
        if (mkfifo(VIDEO_FIFO, 0644) < 0) {
            throw runtime_error("Could not create pipe");
        }

        pipe = fopen(VIDEO_FIFO, "w");
        if (!pipe) {
            throw runtime_error("Could not open pipe for writing (" +
                                to_string(errno) + "): " + strerror(errno));
        }
    }

    /*
     * Close file handle for named pipe and delete it.
     */
    inline void deletePipe()
    {
        if (pipe) {
            fclose(pipe);
        }
        unlink(VIDEO_FIFO);
    }

    /*
     * Fork process and start mplayer (in xterm).
     */
    inline void launchMplayer()
    {
        if ((mplayer = fork()) == 0) {
            execlp("xterm",
                   "xterm",
                   "-e",
                   "mplayer",
                   "-demuxer",
                   "h264es",
                   VIDEO_FIFO,
                   "-benchmark",
                   "-really-quiet",
                   NULL);
            cerr << "Error: Could not start mplayer" << endl;
            exit(1);
        }
    }

    /*
     * Send SIGKILL to mplayer child process.
     */
    inline void killMplayer()
    {
        if (mplayer) {
            kill(mplayer, SIGKILL);
        }
    }

    /*
     * Invoked when we receive a packet containing H264 params.
     */
    static eARCONTROLLER_ERROR configCallback(ARCONTROLLER_Stream_Codec_t codec,
                                              void *data);

    /*
     * Invoked when we receive a packet containing an H264-encoded frame.
     */
    static eARCONTROLLER_ERROR frameCallback(ARCONTROLLER_Frame_t *frame,
                                             void *data);
};
// end BebopVideoStream
} // Robots
} // BoBRobotics

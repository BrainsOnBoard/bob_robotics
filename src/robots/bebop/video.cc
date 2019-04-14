// BoB robotics includes
#include "robots/bebop/bebop.h"
#include "common/logging.h"

namespace BoBRobotics {
namespace Robots {
bool
Bebop::VideoStream::initCodec()
{
    if (m_CodecInitialised) {
        return true;
    }

    try {
        // Very first init
        avcodec_register_all();
        av_register_all();
        av_log_set_level(AV_LOG_QUIET);

        m_CodecPtr = avcodec_find_decoder(AV_CODEC_ID_H264);
        if (!m_CodecPtr) {
            throw std::runtime_error("Codec H264 not found!");
        }

        m_CodecContextPtr = avcodec_alloc_context3(m_CodecPtr);
        m_CodecContextPtr->pix_fmt = AV_PIX_FMT_YUV420P;
        m_CodecContextPtr->skip_frame = AVDISCARD_DEFAULT;
        m_CodecContextPtr->error_concealment = FF_EC_GUESS_MVS | FF_EC_DEBLOCK;
        m_CodecContextPtr->skip_loop_filter = AVDISCARD_DEFAULT;
        m_CodecContextPtr->workaround_bugs = AVMEDIA_TYPE_VIDEO;
        m_CodecContextPtr->codec_id = AV_CODEC_ID_H264;
        m_CodecContextPtr->skip_idct = AVDISCARD_DEFAULT;

        // At the beginning we have no idea about the frame size
        m_CodecContextPtr->width = 0;
        m_CodecContextPtr->height = 0;

        if (m_CodecPtr->capabilities & CODEC_CAP_TRUNCATED) {
            m_CodecContextPtr->flags |= CODEC_FLAG_TRUNCATED;
        }
        m_CodecContextPtr->flags2 |= CODEC_FLAG2_CHUNKS;

        m_FramePtr = av_frame_alloc();
        if (!m_FramePtr) {
            throw std::runtime_error("Cannot allocate memory for frames!");
        }

        if (avcodec_open2(m_CodecContextPtr, m_CodecPtr, nullptr) < 0) {
            throw std::runtime_error("Cannot open the decoder!");
        }

        av_init_packet(&m_Packet);
    } catch (const std::runtime_error &e) {
        LOG_ERROR << "Error: " << e.what();
        reset();
        return false;
    }

    m_CodecInitialised = true;
    m_FirstFrameReceived = false;
    return true;
}

bool
Bebop::VideoStream::reallocateBuffers()
{
    if (!m_CodecInitialised) {
        return false;
    }

    try {
        if (m_CodecContextPtr->width == 0 || m_CodecContextPtr->width == 0) {
            throw std::runtime_error(std::string("Invalid frame size: ") +
                                     std::to_string(m_CodecContextPtr->width) + " x " +
                                     std::to_string(m_CodecContextPtr->width));
        }

        const auto num_bytes = static_cast<uint32_t>(avpicture_get_size(
                AV_PIX_FMT_BGR24, m_CodecContextPtr->width, m_CodecContextPtr->width));
        m_FrameBGRPtr = av_frame_alloc();

        if (!m_FrameBGRPtr) {
            throw std::runtime_error("Cannot allocate memory for frames!");
        }

        m_FrameBGRRawPtr = reinterpret_cast<uint8_t *>(av_malloc(num_bytes * sizeof(uint8_t)));
        if (!m_FrameBGRRawPtr) {
            throw std::runtime_error(std::string("Cannot allocate memory for the buffer: ") +
                                     std::to_string(num_bytes));
        }

        int ret = avpicture_fill(reinterpret_cast<AVPicture *>(m_FrameBGRPtr),
                                 m_FrameBGRRawPtr,
                                 AV_PIX_FMT_BGR24,
                                 m_CodecContextPtr->width,
                                 m_CodecContextPtr->height);
        if (ret == 0) {
            throw std::runtime_error("Failed to initialize the picture data structure.");
        }

        m_ImgConvertContextPtr = sws_getContext(m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                m_CodecContextPtr->pix_fmt,
                                                m_CodecContextPtr->width,
                                                m_CodecContextPtr->height,
                                                AV_PIX_FMT_BGR24,
                                                SWS_FAST_BILINEAR,
                                                nullptr,
                                                nullptr,
                                                nullptr);
    } catch (const std::runtime_error &e) {
        LOG_ERROR << "Error: " << e.what();
        reset(); // reset() is intentional
        return false;
    }

    return true;
}

void
Bebop::VideoStream::cleanupBuffers()
{
    if (m_FrameBGRPtr) {
        av_free(m_FrameBGRPtr);
    }

    if (m_FrameBGRRawPtr) {
        av_free(m_FrameBGRRawPtr);
    }

    if (m_ImgConvertContextPtr) {
        sws_freeContext(m_ImgConvertContextPtr);
    }
}

void
Bebop::VideoStream::reset()
{
    if (m_CodecContextPtr) {
        avcodec_close(m_CodecContextPtr);
    }

    if (m_FramePtr) {
        av_free(m_FramePtr);
    }

    cleanupBuffers();

    m_CodecInitialised = false;
    m_FirstFrameReceived = false;
}

Bebop::VideoStream::VideoStream(Bebop &bebop)
{
    checkError(ARCONTROLLER_Device_SetVideoStreamCallbacks(
            bebop.m_Device.get(), configCallback, frameCallback, nullptr, this));
}

Bebop::VideoStream::~VideoStream()
{
    reset();
}

void
Bebop::VideoStream::convertFrameToBGR()
{
    if (!m_CodecContextPtr->width || !m_CodecContextPtr->height)
        return;
    sws_scale(m_ImgConvertContextPtr,
              m_FramePtr->data,
              m_FramePtr->linesize,
              0,
              m_CodecContextPtr->height,
              m_FrameBGRPtr->data,
              m_FrameBGRPtr->linesize);
}

bool
Bebop::VideoStream::setH264Params(uint8_t *sps_buffer_ptr,
                                  uint32_t sps_buffer_size,
                                  uint8_t *pps_buffer_ptr,
                                  uint32_t pps_buffer_size)
{
    // This function is called in the same thread as decode(), so no sync is
    // necessary
    // TODO: Exact sizes + more error checkings
    m_UpdateCodecParams = (sps_buffer_ptr && pps_buffer_ptr &&
                           sps_buffer_size && pps_buffer_size &&
                           (pps_buffer_size < 32) && (sps_buffer_size < 32));

    if (m_UpdateCodecParams) {
        m_CodecData.resize(sps_buffer_size + pps_buffer_size);
        std::copy(sps_buffer_ptr,
                  sps_buffer_ptr + sps_buffer_size,
                  m_CodecData.begin());
        std::copy(pps_buffer_ptr,
                  pps_buffer_ptr + pps_buffer_size,
                  m_CodecData.begin() + sps_buffer_size);
    } else {
        // invalid data
        m_CodecData.clear();
    }

    return m_UpdateCodecParams;
}

bool
Bebop::VideoStream::decode(const ARCONTROLLER_Frame_t *framePtr)
{
    if (!m_CodecInitialised) {
        if (!initCodec()) {
            LOG_ERROR << "Codec initialization failed!";
            return false;
        }
    }

    /*
     * For VideoStream2, we trick avcodec whenever we receive a new SPS/PPS
     * info from the Bebop. setH264Params() function will fill a buffer with
     * SPS/PPS data, then these are passed to avcodec_decode_video2() here, once
     * for each SPS/PPS update. Apparantly, avcodec_decode_video2() function
     * picks up the changes and apply them to upcoming video packets.
     *
     * More info on VS v2.0:
     * http://developer.parrot.com/blog/2016/ARSDK-3-8-release/
     *
     * */
    if (m_UpdateCodecParams && m_CodecData.size()) {
        m_Packet.data = &m_CodecData[0];
        m_Packet.size = static_cast<int>(m_CodecData.size());
        int32_t frame_finished = 0;
        const int32_t len = avcodec_decode_video2(
                m_CodecContextPtr, m_FramePtr, &frame_finished, &m_Packet);
        if (len >= 0 && len == m_Packet.size) {
            // success, skip this step until next codec update
            m_UpdateCodecParams = false;
        } else {
            LOG_ERROR << "Unexpected error while updating H264 parameters.";
            return false;
        }
    }

    if (!framePtr->data || !framePtr->used) {
        LOG_WARNING << "Invalid frame data. Skipping.";
        return false;
    }

    m_Packet.data = framePtr->data;
    m_Packet.size = static_cast<int>(framePtr->used);

    const int width_prev = getFrameWidth();
    const int height_prev = getFrameHeight();

    int32_t frame_finished = 0;
    while (m_Packet.size > 0) {
        const int32_t len = avcodec_decode_video2(
                m_CodecContextPtr, m_FramePtr, &frame_finished, &m_Packet);
        if (len < 0) {
            return false;
        }
        if (frame_finished) {
            if ((getFrameWidth() != width_prev) ||
                (getFrameHeight() != height_prev)) {
                if (!reallocateBuffers()) {
                    LOG_WARNING << "Buffer reallocation failed!";
                }
            }
            convertFrameToBGR();
        }

        if (m_Packet.data) {
            m_Packet.size -= len;
            m_Packet.data += len;
        }
    }
    return true;
}

cv::Size
Bebop::VideoStream::getOutputSize() const
{
    return cv::Size(856, 480);
}

std::string
Bebop::VideoStream::getCameraName() const
{
    return "bebop";
}

bool
Bebop::VideoStream::readFrame(cv::Mat &frame)
{
    if (m_NewFrame) {
        std::lock_guard<decltype(m_FrameMutex)> guard(m_FrameMutex);
        m_Frame.copyTo(frame);
        return true;
    } else {
        return false;
    }
}

eARCONTROLLER_ERROR
Bebop::VideoStream::configCallback(ARCONTROLLER_Stream_Codec_t codec, void *data)
{
    auto stream = reinterpret_cast<VideoStream *>(data);
    stream->setH264Params(codec.parameters.h264parameters.spsBuffer,
                          static_cast<uint32_t>(codec.parameters.h264parameters.spsSize),
                          codec.parameters.h264parameters.ppsBuffer,
                          static_cast<uint32_t>(codec.parameters.h264parameters.ppsSize));
    return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR
Bebop::VideoStream::frameCallback(ARCONTROLLER_Frame_t *frame, void *data)
{
    auto stream = reinterpret_cast<VideoStream *>(data);
    if (stream->decode(frame)) {
        std::lock_guard<decltype(stream->m_FrameMutex)> guard(stream->m_FrameMutex);
        stream->m_Frame = cv::Mat(stream->getOutputSize(), CV_8UC3, (void *) stream->m_FrameBGRRawPtr);
        stream->m_NewFrame = true;
    }
    return ARCONTROLLER_OK;
}
} // Robots
} // BoBRobotics

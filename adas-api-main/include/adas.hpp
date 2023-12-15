#ifndef ADAS_HPP
#define ADAS_HPP

#include <control_frames.hpp>
#include <status_frames.hpp>

#include <map>

namespace adas_api{
    /**
     * @brief Quickly define and setup all the status and control frames
     *      needed for the ADAS system.
     * 
     * @details If you want to have direct access to the individual frames
     *      then this is the class to use. The individual frames will prevent
     *      sending dangerous values to the vehicle but you are responsible
     *      for managing the correct sequence etc to get the vehicle to 
     *      respond correctly.
     */
    class Adas
    {
    protected:
        const int _canSocket;

    public:
        // incoming frames
        AxleTorqueFrame frontAxle;
        AxleTorqueFrame rearAxle { false };
        BrakeFrame brakes;
        StatusFrame status;
        SteeringFrame steering;
        WheelCountsFrame wheelCounts;
        WheelSpeedsFrame wheelSpeeds;

        // outgoing frames
        BrakeControlFrame brakeControl;
        AxleControlFrame frontControl { this->frontAxle, this->brakes, this->wheelSpeeds, this->status, this->brakeControl };
        AxleControlFrame rearControl { this->rearAxle, this->brakes, this->wheelSpeeds, this->status, this->brakeControl, false };
        StatusControlFrame statusControl { this->frontAxle, this->rearAxle, this->status };
        SteeringControlFrame steeringControl { this->steering, this->status };

        /**
         * @brief A map of all the incoming frames.
         * 
         * @details A map is used with the frame ID as the key to allow 
         *      for fast lookup by the read() method.
        */
        const std::map<const uint32_t, ProcessFrame* const> statusFrames
            {
                { this->frontAxle.frameId, &this->frontAxle },
                { this->rearAxle.frameId, &this->rearAxle },
                { this->brakes.frameId, &this->brakes },
                { this->status.frameId, &this->status },
                { this->steering.frameId, &this->steering },
                { this->wheelCounts.frameId, &this->wheelCounts },
                { this->wheelSpeeds.frameId, &this->wheelSpeeds }
            };

        /**
         * @brief An array of all the outgoing frames.
         */
        const std::array<ControlFrame* const, 5> controlFrames 
            { 
                &this->frontControl, 
                &this->rearControl, 
                &this->brakeControl, 
                &this->statusControl, 
                &this->steeringControl
            };

        /**
         * @brief Attempt to read the specified number of frames from the
         *     CAN bus.
         * 
         * @details The Adas class sets up the CAN socket to be non-blocking
         *     so this method will not block. If there are no frames to read
         *     then the method will return early.
         * 
         * @param n Number of frames to attempt to read.
         */
        virtual void read( const int n=50 );

        /**
         * @brief Write all the control frames to the CAN bus.
         */
        virtual void write();

        /**
         * @brief Construct a new Adas object
         * 
         * @param device The name of the CAN device to use.
         */
        explicit Adas( const std::string& device );

        Adas( const Adas& ) = delete;

        /**
         * @brief Destroy the Adas object
         * 
         * @details The destructor will attempt to close the CAN socket.
         */
        virtual ~Adas();
    };
};

#endif // ADAS_HPP



#ifndef ARDUINO_SERIAL_COMMUNICATOR_H
#define ARDUINO_SERIAL_COMMUNICATOR_H

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


namespace arduino_robot
{

template<class Channel>
class SimpleCommunicator
{

	private:

		enum class CommunicatorReceptionState {AwaitingMessage, GettingMessageLength, ReceivingMessage, MessageReceived};

		static constexpr uint8_t MESSAGE_START_BYTE_ = 255;
		static constexpr uint8_t MAX_MESSAGE_LENGTH_ = 255;
		uint8_t reception_buffer_[MAX_MESSAGE_LENGTH_];

		uint8_t expected_message_length_;
		uint8_t bytes_received_;
		CommunicatorReceptionState reception_state_ = CommunicatorReceptionState::AwaitingMessage;

		Channel& channel_;

		bool seekForIncomingMessage();
		bool getIncomingMessageLength();
		bool receiveIncomingMessage();
		
	public:
		SimpleCommunicator(Channel& channel)
			:channel_(channel) 
		{
			reset();
		};

		void reset();
		void sendMessage(const uint8_t* buffer, uint8_t length);
		void sendMessage(uint8_t header_byte, const uint8_t * buffer, uint8_t length);
		const uint8_t* getReceivedMessage() const;
		uint8_t getReceivedMessageLength() const;
		void runReceptionLoop();
};


template<class Channel>
bool SimpleCommunicator<Channel>::seekForIncomingMessage()
{
	while (channel_.available()) 
    {
		if (channel_.read() == MESSAGE_START_BYTE_) 
        {
			reception_state_ = CommunicatorReceptionState::GettingMessageLength;
			return true;
		}
	}

	return false;
}

template<class Channel>
bool SimpleCommunicator<Channel>::getIncomingMessageLength()
{
	if (channel_.available()) 
    {
		expected_message_length_ = channel_.read();
		bytes_received_ = 0;
		reception_state_ = CommunicatorReceptionState::ReceivingMessage;
		return true;
	}
    
	return false;
}

template<class Channel>
bool SimpleCommunicator<Channel>::receiveIncomingMessage()
{
	int bytes_to_receive = expected_message_length_ - bytes_received_;

	if (channel_.available() && bytes_to_receive > 0) 
    {
		bytes_received_ += channel_.readBytes(reception_buffer_ + bytes_received_, bytes_to_receive);
	}

	if (bytes_received_ >= expected_message_length_) 
    {
		reception_state_ = CommunicatorReceptionState::MessageReceived;
        return true;
	}

	return false;
}

template<class Channel>
void SimpleCommunicator<Channel>::reset()
{
	expected_message_length_ = 0;
	bytes_received_ = 0;
	reception_state_ = CommunicatorReceptionState::AwaitingMessage;
}

template<class Channel>
void SimpleCommunicator<Channel>::sendMessage(const uint8_t * buffer, uint8_t length)
{
    channel_.flush();
	uint8_t preamble = MESSAGE_START_BYTE_;
	channel_.write(&preamble, 1);
	channel_.write(&length, 1);
	channel_.write(buffer, length);
}


template<class Channel>
void SimpleCommunicator<Channel>::sendMessage(uint8_t header_byte, const uint8_t * buffer, uint8_t length)
{
    channel_.flush();
	uint8_t preamble = MESSAGE_START_BYTE_;
	channel_.write(&preamble,1);
	length++;
	channel_.write(&length, 1);
    channel_.write(&header_byte,1);
	channel_.write(buffer, length - 1);
}

template<class Channel>
const uint8_t * SimpleCommunicator<Channel>::getReceivedMessage() const
{
	if (reception_state_ != CommunicatorReceptionState::MessageReceived) 
    {
		return nullptr;
	}
	else 
    {
		return reception_buffer_;
	}
}

template<class Channel>
uint8_t SimpleCommunicator<Channel>::getReceivedMessageLength() const
{
	if (reception_state_!= CommunicatorReceptionState::MessageReceived) 
    {
		return 0;
	}
	else 
    {
		return expected_message_length_;
	}
}

template<class Channel>
void SimpleCommunicator<Channel>::runReceptionLoop()
{
	bool res = true;
	while (res) 
    {  
		switch (reception_state_) 
        {
            case CommunicatorReceptionState::AwaitingMessage:
                res = seekForIncomingMessage();
                break;
            case CommunicatorReceptionState::GettingMessageLength:
                res = getIncomingMessageLength();
                break;
            case CommunicatorReceptionState::ReceivingMessage:
                receiveIncomingMessage();
                res = false; //exit since we either read all available bytes or entire message
                break;
            case CommunicatorReceptionState::MessageReceived:
                res = false;
                break;
		}
	}
}
}


#endif
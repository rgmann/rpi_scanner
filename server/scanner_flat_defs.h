#ifndef  SCANNER_FLAT_DEFS
#define  SCANNER_FLAT_DEFS


namespace  scanner_flat_defs {

	const uint8_t MARKER[] = {0xDE,0xAD,0xBE,0xEF};

	enum packet_types {
		SET_MODE = 1,
		STATUS_REQUEST
		STATUS_RESPONSE = 2,
		POINT = 3
	};

	enum scanner_mode {
		MODE_IDLE = 0,
		MODE_POINT = 1,
		MODE_RASTER = 2
	};

	struct message_header {
		uint32_t	marker;
		uint32_t size;
		uint32_t type;
	};

	struct message_timestamp {
		uint32_t sec;
		uint32_t usec;
	};

	struct scanner_mode {
		uint32_t mode;
		float		min_phi;
		float		max_phi;
		float		min_theta;
		float		max_theta;
		float		phi;
		float		theta;
	};

	struct scanner_point {
		message_timestamp timestamp;
		float 	phi;
		float 	theta;
		float		r;
	};

	struct scanner_status {
		uint32_t mode;
		float		min_phi;
		float		max_phi;
		float		min_theta;
		float		max_theta;
		float		phi;
		float		theta;
		bool 		ll_eye_safe;
		bool     ll_good;
		uint8_t  ll_status;
		uint8_t	pad;
		struct time {
			uint32_t sec;
			uint32_t usec;
		} timestamp;
	};

	struct message {
		message_header header;
		union {
			scanner_mode 	mode;
			scanner_status status;
			scanner_point 	point;
		} data;
	};

	size_t full_message_size( const message& message ) {
		return ( message.header.size + sizeof(message_header) );
	}
} 

#endif // SCANNER_FLAT_DEFS

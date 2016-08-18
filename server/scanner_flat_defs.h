#ifndef  SCANNER_FLAT_DEFS
#define  SCANNER_FLAT_DEFS


namespace  scanner_flat_defs {

	const uint8_t MARKER[] = {0xEF,0xBE,0xAD,0xDE};

	enum packet_types {
		SET_MODE = 1,
		SET_LIMITS = 2,
		STATUS_REQUEST = 3,
		STATUS_RESPONSE = 4,
		POINT = 5,
		SHUTDOWN = 6
	};

	enum scanner_modes {
		MODE_IDLE = 0,
		MODE_POINT = 1,
		MODE_RASTER = 2
	};

	struct __attribute__((packed)) message_header {
		uint32_t	marker;
		uint32_t size;
		uint32_t type;
	};

	struct __attribute__((packed)) message_timestamp {
		uint32_t sec;
		uint32_t usec;
	};

	struct __attribute__((packed)) scanner_mode {
		uint32_t mode;
		float		phi;
		float		theta;
	};

	struct __attribute__((packed)) scanner_limits {
		float		min_phi;
		float		max_phi;
		float		min_theta;
		float		max_theta;
	};

	struct __attribute__((packed)) scanner_shutdown {
		bool reboot;
		uint8_t pad[3];
	};

	struct __attribute__((packed)) scanner_point {
		message_timestamp timestamp;
		float 	phi;
		float 	theta;
		float		r;
	};

	struct __attribute__((packed)) scanner_status {
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

	struct __attribute__((packed)) message {
		message_header header;
		union {
			scanner_mode 	mode;
			scanner_status status;
			scanner_point 	point;
			scanner_limits limits;
			scanner_shutdown shutdown;
		} data;
	};

	size_t full_message_size( const message& message ) {
		return ( message.header.size + sizeof(message_header) );
	}
} 

#endif // SCANNER_FLAT_DEFS

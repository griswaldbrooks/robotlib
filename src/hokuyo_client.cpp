#include 	"hokuyo_client.hpp"
#define 	_USE_MATH_DEFINES
#include 	<cmath>
#include 	<boost/date_time/posix_time/posix_time.hpp>

namespace hokuyo
{
	typedef		union { uint8_t bytes[8UL]; double fpn; } fpn4b_t;

	#define		HOKUYO_START_SERVICE 		(0x1)
	#define		HOKUYO_REQUEST_SEND 		(0x2)
	#define		HOKUYO_END_SERVICE 			(0xF)
	#define 	HOKUYO_ANGLE_SHIFT			(2.268928027)
	#define 	HOKUYO_FRAME_SIZE 			(628UL)
	#define 	HOKUYO_MAX_RANGE 			5.600f //meters
	#define 	HOKUYO_MIN_RANGE 			0.020f //meters

	#if HOKUYO_DEBUG
	#define 	HOKUYO_MESSAGE(msg)			std::cout << "HOKUYO  :  " << msg << std::endl
	#else
	#define 	HOKUYO_MESSAGE(msg)
	#endif


	float tcp_client::get_max_range() {return HOKUYO_MAX_RANGE;}
	float tcp_client::get_min_range() {return HOKUYO_MIN_RANGE;}

	std::ostream& operator<<( std::ostream& os, tcp_client& cli)
	{	
		// Print Size
		if(cli.valid())
		{
			// Print Angle0,Distance0,Angle1,Distance1,...,AngleN,DistanceN
			for(size_t idx = 0UL; idx < cli.size(); idx++)
				os << cli.data[0][idx] << "  " << cli.data[1][idx] << "  ";
		}
		return os;
	}


	bool tcp_client::check()
	{
		return true;///check for bad read TBIMPL
	}


	void tcp_client::putb( uint8_t byte )
	{	
		socket.write_some(boost::asio::buffer(&byte,1UL));
	}


	uint8_t tcp_client::getb()
	{
		uint8_t byte;
		while(!socket.read_some(boost::asio::buffer(&byte,1UL)));
		return byte;
	}


	void tcp_client::connect()
	{
		HOKUYO_MESSAGE("Connecting...");

		/// Setup connection resolution stuff
		asio::ip::tcp::resolver 			resolver(socket.get_io_service());
		asio::ip::tcp::resolver::query 		query(address.c_str(), port.c_str());
		asio::ip::tcp::endpoint 			endpt = *resolver.resolve(query);

		/// Setup connection through socket
		socket.connect(endpt);

		HOKUYO_MESSAGE("Connected on ::");
		HOKUYO_MESSAGE("-------------::");
		HOKUYO_MESSAGE("          IP :: "<<socket.remote_endpoint().address().to_string());
		HOKUYO_MESSAGE("        PORT :: "<<socket.remote_endpoint().port());
		HOKUYO_MESSAGE("-------------::");

	}


	void tcp_client_read_service( tcp_client* cli )
	{
		
		/// Initiate serial on server side
		cli->putb(HOKUYO_START_SERVICE);
		HOKUYO_MESSAGE("Starting aquisition...");

		/// Run service as long as thread lives
		for(;;)
		{	

			/// Sleep
			boost::this_thread::sleep(boost::posix_time::milliseconds(50));

			/// Check if read requested
			if(cli->requested)
			{
				/// Initiate transimission by request
				HOKUYO_MESSAGE("Requesting data...");
				cli->putb(HOKUYO_REQUEST_SEND);
		
				/// Data is not ready
				cli->requested = false;
				cli->available = false;

				/// Clear old data
				cli->clear();

				/// Begin reading frame
				for(size_t item = 0UL; item < 2U; item++ )
				{
					uint8_t msb, lsb, byte;
					fpn4b_t f_share_b;
					size_t	rec_size, jdx = 0;

					/// Get size field
					msb		= cli->getb();
					lsb		= cli->getb();
					rec_size= ( (size_t)msb | ( (size_t)lsb << 8U ) );

					/// Get data bytes + convert to floats
					for( size_t idx = 0; idx < (rec_size); ++idx)
					{
						/// Convert byte stream to floats
						f_share_b.bytes[jdx++] = cli->getb();

						/// Push data to angle/distance vectors
						if(jdx==8)
						{
							jdx = 0UL;

							// When we are talking "angles" shift them
							if(item==0UL)
								cli->data[item].push_back(f_share_b.fpn - HOKUYO_ANGLE_SHIFT);
							else//you were shifting the range data ya dumby
								cli->data[item].push_back(f_share_b.fpn);
						}
					}
				}

				/// Check the state of transmission
				/// * There should be no bytes avail if a request was fully processed
				if(cli->check())
				{
					++cli->n_rec_frames;
					HOKUYO_MESSAGE("Frames : "<<cli->n_rec_frames);
					HOKUYO_MESSAGE("Data   : "<<size());
				}
				else
				{
					HOKUYO_MESSAGE("Bad frame!");
					++cli->n_bad_frames;
				}
			}

			/// Data is ready
			cli->available = true;
		}
	}


	void tcp_client::request()
	{
		requested = true;
	}


	bool tcp_client::valid()
	{
		return avail() && (size()>0);

	}


	bool tcp_client::avail()
	{
		return available;
	}


	const size_t& tcp_client::get_frame_count()
	{
		return n_rec_frames;
	}


	const size_t& tcp_client::get_missed_count()
	{
		return n_bad_frames;
	}


	const size_t tcp_client::size()						
	{ 
		return data[0UL].size(); 
	}


	const fbuffer& tcp_client::get_angles() 		
	{ 
		return data[0UL]; 
	}

	
	const fbuffer& tcp_client::get_distances() 		
	{ 	
		return data[1UL]; 
	}
	

	const float& tcp_client::get_angle( const size_t idx ) 		
	{ 
		return data[0UL][idx]; 
	}

	
	const float& tcp_client::get_distance( const size_t idx ) 		
	{ 	
		return data[1UL][idx]; 
	}

	
	void  tcp_client::clear() 
	{ 
		data[0UL].clear(); 
		data[1UL].clear();
	}


	void	tcp_client::start()
	{
		/// If a job is not active...
		if(job==NULL)
		{
			connect();

			HOKUYO_MESSAGE("Starting job!");
			job = new thread(tcp_client_read_service,this);
			job->detach();
		}
	}


	void	tcp_client::stop()
	{
		/// If a job has been started..
		if(job!=NULL)
		{
			HOKUYO_MESSAGE("Stopping job!");

			job->interrupt();
			delete job;

			/// Reset frame counters
			n_rec_frames = n_bad_frames = 0UL;
			
			/// Cleanup the service on server side
			putb(HOKUYO_END_SERVICE);
		}
	}


	tcp_client::tcp_client( const char* address, const char* port, asio::ip::tcp::socket& socket ) :
		address(address)	,
		port(port)			,
		socket(socket)		,
		n_rec_frames(0UL)	,
		n_bad_frames(0UL)	,
		requested(false)	,
		available(false)	,
		job(NULL)
	{
		HOKUYO_MESSAGE("Creating handle");
	}


	tcp_client::~tcp_client()
	{
		stop();
	}
};
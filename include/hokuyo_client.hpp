/// @file 	This code works with the Hokuyo Python Client
///	@brief 	
#ifndef HOKUYO_CLIENT_HPP
#define HOKUYO_CLIENT_HPP 1

#include<stdio.h>
#include<stdint.h>
#include<vector>
#include<string>
#include<boost/thread.hpp>
#include<boost/asio.hpp>
#include<boost/array.hpp>




namespace hokuyo
{
	using namespace boost;
	using namespace std;

	#define HOKUYO_DEBUG 			0			
	typedef std::vector<float>		fbuffer;

	




	/// @brief Hokuyo laser client
	class tcp_client
	{
	friend
		std::ostream& operator<<( std::ostream& os, tcp_client& cli);
	friend 
		void tcp_client_read_service( tcp_client* cli );
	private:
		asio::ip::
			tcp::socket& 	socket;							///< associated communications socket
		system::error_code	socket_error;					///< error code
		
		const std::string 	address;						///< ip-address of server
		const std::string  	port;							///< port to server
		
		bool 				requested;						///< request for new data
		bool 				available;						///< flag data is available
		size_t				n_rec_frames;					///< number of recieved frames
		size_t				n_bad_frames;					///< number of frames w/ errors
		thread*				job;							///< thread for read process
		fbuffer				data[2UL];						///< angle/distance data buffers

		void				clear();						///< clears all data buffers
		bool				check();						///< checks that socket buf is empty 
		void				putb( uint8_t byte );			///< sends single byte thru socket
		uint8_t				getb();							///< gets single byte thru socket
 		void				connect();						///< sets up socket connections

	public:
		const	size_t		size();							///< returns the number of available angle/distance pairs
		const	fbuffer&	get_angles();					///< returns vector of angle 	(valid if avail()==true)
		const	fbuffer&	get_distances();				///< returns vector distances 	(valid if avail()==true)
		const	float&		get_angle(const size_t idx);	///< returns (idx)th angle
		const	float&		get_distance(const size_t idx);	///< returns (jdx)th distance

		const	size_t&		get_frame_count();				///< returns the number of recieved frames
		const	size_t&		get_missed_count();				///< returns the number of incomplete/faulty frames
		float				get_max_range();
		float				get_min_range();

		void				start();						///< starts server communication process
		void				stop();							///< ends server communication process
		void 				request();						///< request a new aquisition
		bool 				avail();						///< returns true if 'data-flag' is set
		bool 				valid();						///< returns true if avail() and buffer has correct number of elements
		

		/// @brief	Default constructor; sets up connection to client.
		/// @param	[in]	address		server's ip address
		/// @param	[in]	port		commincation port
		/// @param	[in		socket		socket object thru which comms are performed
		explicit tcp_client( const char* address, const char* port, asio::ip::tcp::socket& socket );


		/// @brief	Closes socket comms upon deletion
		virtual ~tcp_client();
	};




	/// @ Some utils for use with the client object
	namespace utils
	{	

		/// pointTy must have public members 'x' and 'y'
		template<typename pointTy>
			void to_cart( tcp_client& hok, std::vector<pointTy>& _out )
			{
				if(_out.size() != hok.size())
					_out.resize(hok.size());
				
				for( size_t idx = 0UL; idx < hok.size(); idx++ )
				{
					_out[idx].x = sin( -hok.get_angles()[idx] )*hok.get_distances()[idx];
					_out[idx].y = cos( -hok.get_angles()[idx] )*hok.get_distances()[idx];
				}
			}


		template<typename fTy>
			void to_cart( tcp_client& hok, std::vector<fTy>& _X, std::vector<fTy>& _Y )
			{
				if(_X.size() != hok.size())
					_X.resize(hok.size());
				
				if(_Y.size() != hok.size())
					_Y.resize(hok.size());
				
				for( size_t idx = 0UL; idx < hok.size(); idx++ )
				{
					_X[idx] = (sin( -hok.get_angles()[idx] )*hok.get_distances()[idx]);
					_Y[idx] = (cos( -hok.get_angles()[idx] )*hok.get_distances()[idx]);
				}
			}
	}


};//hokuyo
#endif
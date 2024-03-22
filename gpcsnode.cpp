#include "gpcsnode.h"
#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

gpcs::gpcsnode::gpcsnode() :io_context_()
{
	char* configFile = std::getenv("GPCS_CONFIG_XML_PATH");
	if (configFile) {
		std::cout << "Config file path: " << configFile << std::endl;
		// Now you can use configFile to access the config.xml file
	}
	else {
		std::cerr << "Environment variable CONFIG_XML_PATH is not set" << std::endl;
	}

	boost::property_tree::ptree pt;
	try {
		boost::property_tree::read_xml(configFile, pt);
	}
	catch (boost::property_tree::xml_parser_error& e) {
		std::cerr << "Failed to load config.xml: " << e.what() << std::endl;
		return ;
	}
	try {
		coreURL = pt.get<int>("parameters.coreportnum.value");
		USE_MAX_DATA_LENGTH= pt.get<int>("parameters.databufferlength.value");
	}
	catch (boost::property_tree::ptree_bad_path& e) {
		std::cerr << "Invalid XML format: " << e.what() << std::endl;
		return ;
	}
}

void gpcs::gpcsnode::init(std::string nodename)
{

	Core_socket = new boost::asio::ip::tcp::socket(io_context_);
	boost::asio::ip::tcp::resolver resolver(io_context_);
	boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve("localhost", std::to_string(coreURL));

	// Connect to the server
	boost::asio::connect(*Core_socket, endpoints);
	//registername
	std::string cmd = "register " + nodename +" \n ";
	Core_socket->write_some(boost::asio::buffer(cmd));
	StartRead();
	boost::this_thread::sleep(boost::posix_time::seconds(2));

	boost::thread Iothread(boost::bind(&gpcs::gpcsnode::runIoContext, this));

	return;
}

void gpcs::gpcsnode::StartRead()
{
	Core_socket->async_read_some(boost::asio::buffer(data_, max_length),
		boost::bind(&gpcs::gpcsnode::handleMasterRead, this,
			boost::asio::placeholders::error,
			boost::asio::placeholders::bytes_transferred));
}



void gpcs::gpcsnode::execute(std::string cmd)
{
	Core_socket->write_some(boost::asio::buffer(cmd));
}

void gpcs::gpcsnode::callCallback(const std::string& topicname, std::string input)
{
	if (CallbackMap.find(topicname) != CallbackMap.end())
	{
		CallbackMap[topicname](input);
	}
	else
	{
		std::cout << "Callback for topicname " << topicname << " not found." << std::endl;
	}
}

void gpcs::gpcsnode::runIoContext()
{
	io_context_.run();
}

void gpcs::gpcsnode::spinonce()
{
	bool sigstart;
	session_group_mutex.lock();
	for (auto it : sessiongroup)
	{
		if (it->IsSubscriber = true)
		{
			sigstart = true;
		}
		else
		{
			sigstart = false;
		}
		session* callbacksession = it;
		if (sigstart)
		{
			callbacksession->queueMutex.lock();
			
			if (callbacksession->dataQueue.empty())
			{
			}
			else 
			{
				std::string topicName= callbacksession->related_topicname;
				std::string somedatavalue = callbacksession->dataQueue.front();
				callbacksession->dataQueue.pop();
				callCallback(topicName, somedatavalue);
			}
			callbacksession->queueMutex.unlock();
		}
	}
	session_group_mutex.unlock();
}

void gpcs::gpcsnode::spinonce(int ms)
{
	spinonce();
	boost::this_thread::sleep(boost::posix_time::milliseconds(ms));
}

void gpcs::gpcsnode::parse_command(std::string cmd)
{

	std::vector<std::string> cmdtokens;
	std::istringstream iss(cmd);
	std::string token;

	// Use std::istringstream to split the command
	while (iss >> token) {
		cmdtokens.push_back(token);
	}
	if (cmdtokens[0] == "server" || cmdtokens[0] == "client")
	{
		std::string portnum = cmdtokens[1];
		std::string charecter = cmdtokens[2];
		std::string topicname = cmdtokens[3];
		session* new_session = new session(io_context_);
		if (cmdtokens[0] == "server")
		{
			boost::asio::ip::tcp::acceptor acceptor_(io_context_,
				boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), stoi(portnum)));
			acceptor_.accept(new_session->socket());
			new_session->start();
		}
		else
		{
			boost::asio::ip::tcp::resolver resolver(io_context_);
			boost::asio::ip::tcp::resolver::results_type endpoints = resolver.resolve("localhost",
				portnum);
			// Connect to the server
			boost::asio::connect(new_session->socket(), endpoints);
			new_session->start();
		}
		new_session->related_topicname = topicname;
		if (charecter == "publisher")
		{
			new_session->IsSubscriber = false;
			new_session->IsPublisher = true;
				Publisher* pub = topicname_2_Publisher_map[topicname];
				pub->lock();
				pub->session_group.push_back(new_session);
				pub->unlock();
		}
		else {
			if (topicname_2_sessionindex_map.count(topicname) > 0)
			{
				for (auto it = topicname_2_sessionindex_map.begin(); it != topicname_2_sessionindex_map.end(); ++it)
				{
					sessiongroup[it->second-1]->IsPublisher= false;
					sessiongroup[it->second-1]->IsSubscriber = false;
				}
				new_session->IsSubscriber = true;
				new_session->IsPublisher = false;
			}
			else			{
				new_session->IsSubscriber = true;
				new_session->IsPublisher = false;
			}


		}
		session_group_mutex.lock();
		sessiongroup.push_back(new_session);
		session_group_mutex.unlock();
		int index = sessiongroup.size();
		sessionindex_2_topicname_map.insert(std::make_pair(index, topicname));
		topicname_2_sessionindex_map.insert(std::make_pair(topicname, index));


		return;
	}
}


void gpcs::gpcsnode::handleMasterRead(const boost::system::error_code& error, std::size_t bytes_transferred)
{
	if (!error)
	{
		std::string command(data_);
		memset(data_, 0x00, max_length);
		parse_command(command);

		Core_socket->async_read_some(boost::asio::buffer(data_, max_length),
			boost::bind(&gpcs::gpcsnode::handleMasterRead, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}
	else {
		// Handle error
	}
}

gpcs::Publisher* gpcs::gpcsnode::advertise(std::string topicname)
{
	std::string cmd = "publish " + topicname+" \n";
	publish_advertise_signal = false;

	Publisher* puber = new Publisher;
	topicname_2_Publisher_map.insert(std::make_pair(topicname, puber));
	execute(cmd);
	boost::this_thread::sleep(boost::posix_time::seconds(2));
	return puber;
}

gpcs::Subscriber* gpcs::gpcsnode::subscribe(std::string topicname, std::function<void(std::string)> callback)
{
	CallbackMap[topicname] = [callback](std::string input)
	{
		callback(input);
	};

	std::string cmd = "subscribe " + topicname+" \n";
	execute(cmd);

	boost::this_thread::sleep(boost::posix_time::seconds(2));//


	return nullptr;
}


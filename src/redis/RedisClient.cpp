/**
 * RedisClient.cpp
 *
 * Author: Toki Migimatsu
 *         Shameek Ganguly
 * Created: April 2017
 */

#include "RedisClient.h"
#include <iostream>
#include <sstream>

using namespace std;

void RedisClient::connect(const std::string& hostname, const int port,
	                      const struct timeval& timeout) {
	// Connect to new server
	context_.reset(nullptr);
	redisContext *c= redisConnectWithTimeout(hostname.c_str(), port, timeout);
	std::unique_ptr<redisContext, redisContextDeleter> context(c);

	// Check for errors
	if (!context)
		throw std::runtime_error("RedisClient: Could not allocate redis context.");
	if (context->err)
		throw std::runtime_error("RedisClient: Could not connect to redis server: " + std::string(context->errstr));

	// Save context
	context_ = std::move(context);
}

std::unique_ptr<redisReply, redisReplyDeleter> RedisClient::command(const char *format, ...) {
	va_list ap;
	va_start(ap, format);
	redisReply *reply = (redisReply *)redisvCommand(context_.get(), format, ap);
	va_end(ap);
	return std::unique_ptr<redisReply, redisReplyDeleter>(reply);
}

void RedisClient::ping() {
	auto reply = command("PING");
	std::cout << std::endl << "RedisClient: PING " << context_->tcp.host << ":" << context_->tcp.port << std::endl;
	if (!reply) throw std::runtime_error("RedisClient: PING failed.");
	std::cout << "Reply: " << reply->str << std::endl << std::endl;
}

std::string RedisClient::get(const std::string& key) {
	// Call GET command
	auto reply = command("GET %s", key.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR || reply->type == REDIS_REPLY_NIL)
		throw std::runtime_error("RedisClient: GET '" + key + "' failed.");
	if (reply->type != REDIS_REPLY_STRING)
		throw std::runtime_error("RedisClient: GET '" + key + "' returned non-string value.");

	// Return value
	return reply->str;
}

void RedisClient::set(const std::string& key, const std::string& value) {
	// Call SET command
	auto reply = command("SET %s %s", key.c_str(), value.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: SET '" + key + "' '" + value + "' failed.");
}

void RedisClient::del(const std::string& key) {
	// Call DEL command
	auto reply = command("DEL %s", key.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: DEL '" + key + "' failed.");
}

bool RedisClient::exists(const std::string& key) {
	// Call GET command
	auto reply = command("EXISTS %s", key.c_str());

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR || reply->type == REDIS_REPLY_NIL)
		throw std::runtime_error("RedisClient: EXISTS '" + key + "' failed.");
	if (reply->type != REDIS_REPLY_INTEGER)
		throw std::runtime_error("RedisClient: EXISTS '" + key + "' returned non-integer value.");
	
	bool return_value = (reply->integer == 1);

	if (!return_value && (reply->integer != 0))
	{
		throw std::runtime_error("RedisClient: EXISTS '" + key + "' returned unexpected value (not 0 or 1)");
	}

	return return_value;
}

std::vector<std::string> RedisClient::pipeget(const std::vector<std::string>& keys) {
	// Prepare key list
	for (const auto& key : keys) {
		redisAppendCommand(context_.get(), "GET %s", key.c_str());
	}

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < keys.size(); i++) {
		redisReply *r;
		if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
			throw std::runtime_error("RedisClient: Pipeline GET command failed for key:" + keys[i] + ".");
		
		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type != REDIS_REPLY_STRING)
			throw std::runtime_error("RedisClient: Pipeline GET command returned non-string value for key: " + keys[i] + ".");

		values.push_back(reply->str);
	}
	return values;
}

void RedisClient::pipeset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key list
	for (const auto& keyval : keyvals) {
		redisAppendCommand(context_.get(), "SET %s %s", keyval.first.c_str(), keyval.second.c_str());
	}

	for (size_t i = 0; i < keyvals.size(); i++) {
		redisReply *r;
		if (redisGetReply(context_.get(), (void **)&r) == REDIS_ERR)
			throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");

		std::unique_ptr<redisReply, redisReplyDeleter> reply(r);
		if (reply->type == REDIS_REPLY_ERROR)
			throw std::runtime_error("RedisClient: Pipeline SET command failed for key: " + keyvals[i].first + ".");
	}
}

std::vector<std::string> RedisClient::mget(const std::vector<std::string>& keys) {
	// Prepare key list
	std::vector<const char *> argv = {"MGET"};
	for (const auto& key : keys) {
		argv.push_back(key.c_str());
	}

	// Call MGET command with variable argument formatting
	redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
	std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

	// Check for errors
	if (!reply || reply->type != REDIS_REPLY_ARRAY)
		throw std::runtime_error("RedisClient: MGET command failed.");

	// Collect values
	std::vector<std::string> values;
	for (size_t i = 0; i < reply->elements; i++) {
		if (reply->element[i]->type != REDIS_REPLY_STRING)
			throw std::runtime_error("RedisClient: MGET command returned non-string values.");

		values.push_back(reply->element[i]->str);
	}
	return values;
}

void RedisClient::mset(const std::vector<std::pair<std::string, std::string>>& keyvals) {
	// Prepare key-value list
	std::vector<const char *> argv = {"MSET"};
	for (const auto& keyval : keyvals) {
		argv.push_back(keyval.first.c_str());
		argv.push_back(keyval.second.c_str());
	}

	// Call MSET command with variable argument formatting
	redisReply *r = (redisReply *)redisCommandArgv(context_.get(), argv.size(), &argv[0], nullptr);
	std::unique_ptr<redisReply, redisReplyDeleter> reply(r);

	// Check for errors
	if (!reply || reply->type == REDIS_REPLY_ERROR)
		throw std::runtime_error("RedisClient: MSET command failed.");
}


void RedisClient::createReadCallback(const int callback_number)
{
	int n = _read_callback_indexes.size();
	bool found = false;
	for(int callback_index=0 ; callback_index < n ; callback_index++)
	{
		if(_read_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
	}
	if(found)
	{
		cout << "read callback already exists with this index. Not creating a new one" << endl;
		return;
	}

	_read_callback_indexes.push_back(callback_number);
	_keys_to_read.push_back(vector<string>());
	_objects_to_read.push_back(vector<void *>());
	_objects_to_read_types.push_back(vector<RedisSupportedTypes>());
}

void RedisClient::createWriteCallback(const int callback_number)
{
	int n = _write_callback_indexes.size();
	bool found = false;
	for(int callback_index=0 ; callback_index < n ; callback_index++)
	{
		if(_write_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
	}
	if(found)
	{
		cout << "write callback already exists with this index. Not creating a new one" << endl;
		return;
	}

	_write_callback_indexes.push_back(callback_number);
	_keys_to_write.push_back(vector<string>());
	_objects_to_write.push_back(vector<void *>());
	_objects_to_write_types.push_back(vector<RedisSupportedTypes>());
	_objects_to_write_sizes.push_back(vector<pair<int, int>>());
}



void RedisClient::addDoubleToReadCallback(const int callback_number, const std::string& key, double &object)
{
	int n = _read_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_read_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no read callback with this index in RedisClient::addDoubleToReadCallback(const int callback_number, const std::string& key, double &object)\n");
	}


	_keys_to_read[callback_index].push_back(key);
	_objects_to_read[callback_index].push_back(&object);
	_objects_to_read_types[callback_index].push_back(DOUBLE_NUMBER);
}

void RedisClient::addStringToReadCallback(const int callback_number, const std::string& key, std::string &object)
{
	int n = _read_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_read_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no read callback with this index in RedisClient::addStringToReadCallback(const int callback_number, const std::string& key, std::string &object)\n");
	}

	_keys_to_read[callback_index].push_back(key);
	_objects_to_read[callback_index].push_back(&object);
	_objects_to_read_types[callback_index].push_back(STRING);
}

void RedisClient::addIntToReadCallback(const int callback_number, const std::string& key, int &object)
{
	int n = _read_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_read_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no read callback with this index in RedisClient::addIntToReadCallback(const int callback_number, const std::string& key, int &object)\n");
	}

	_keys_to_read[callback_index].push_back(key);
	_objects_to_read[callback_index].push_back(&object);
	_objects_to_read_types[callback_index].push_back(INT_NUMBER);
}



void RedisClient::addDoubleToWriteCallback(const int callback_number, const std::string& key, double &object)
{
	int n = _write_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_write_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no write callback with this index in RedisClient::addDoubleToWriteCallback(const int callback_number, const std::string& key, double &object)\n");
	}

	_keys_to_write[callback_index].push_back(key);
	_objects_to_write[callback_index].push_back(&object);
	_objects_to_write_types[callback_index].push_back(DOUBLE_NUMBER);
	_objects_to_write_sizes[callback_index].push_back(std::make_pair(0,0));
}

void RedisClient::addStringToWriteCallback(const int callback_number, const std::string& key, std::string &object)
{
	int n = _write_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_write_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no write callback with this index in RedisClient::addStringToWriteCallback(const int callback_number, const std::string& key, std::string &object)\n");
	}

	_keys_to_write[callback_index].push_back(key);
	_objects_to_write[callback_index].push_back(&object);
	_objects_to_write_types[callback_index].push_back(STRING);
	_objects_to_write_sizes[callback_index].push_back(std::make_pair(0,0));
}

void RedisClient::addIntToWriteCallback(const int callback_number, const std::string& key, int &object)
{
	int n = _write_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_write_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no write callback with this index in RedisClient::addIntToWriteCallback(const int callback_number, const std::string& key, int &object)\n");
	}

	_keys_to_write[callback_index].push_back(key);
	_objects_to_write[callback_index].push_back(&object);
	_objects_to_write_types[callback_index].push_back(INT_NUMBER);
	_objects_to_write_sizes[callback_index].push_back(std::make_pair(0,0));
}



void RedisClient::executeReadCallback(const int callback_number)
{
	int n = _read_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_read_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no read callback with this index in RedisClient::executeReadCallback(const int callback_number)\n");
	}

	std::vector<std::string> return_values = pipeget(_keys_to_read[callback_index]);

	for(int i=0 ; i<return_values.size() ; i++)
	{
		switch(_objects_to_read_types[callback_index].at(i))
		{
			case DOUBLE_NUMBER :
			{
				double* tmp_pointer = (double*) _objects_to_read[callback_index].at(i);
				*tmp_pointer = stod(return_values[i]);
			}
			break;

			case INT_NUMBER :
			{
				int* tmp_pointer = (int*) _objects_to_read[callback_index].at(i);
				*tmp_pointer = stoi(return_values[i]);				
			}
			break;

			case STRING :
			{
				std::string* tmp_pointer = (std::string*) _objects_to_read[callback_index].at(i);
				*tmp_pointer = return_values[i];
			}
			break;

			case EIGEN_OBJECT :
			{
				double* tmp_pointer = (double*) _objects_to_read[callback_index].at(i);

				Eigen::MatrixXd tmp_return_matrix = RedisClient::decodeEigenMatrixJSON(return_values[i]);

				int nrows = tmp_return_matrix.rows();
				int ncols = tmp_return_matrix.cols();

				for(int k=0 ; k<nrows ; k++)
				{
					for(int l=0 ; l<ncols ; l++)
					{
						tmp_pointer[k + ncols*l] = tmp_return_matrix(k,l);
					}
				}
			}
			break;

			default :
			break;
		}
	}
}

void RedisClient::executeWriteCallback(const int callback_number)
{
	int n = _write_callback_indexes.size();
	int callback_index = 0;
	bool found = false;
	while(callback_index < n)
	{
		if(_write_callback_indexes[callback_index] == callback_number)
		{
			found = true;
			break;
		}
		callback_index++;
	}
	if(!found)
	{
		throw runtime_error("no write callback with this index in RedisClient::executeWriteCallback(const int callback_number)\n");
	}


	std::vector<std::pair<std::string,std::string>> write_key_value_pairs;

	for(int i=0 ; i<_keys_to_write[callback_index].size() ; i++)
	{
		std::string encoded_value = "";

		switch(_objects_to_write_types[callback_index].at(i))
		{
			case DOUBLE_NUMBER:
			{
				double* tmp_pointer = (double*) _objects_to_write[callback_index].at(i);
				encoded_value = std::to_string(*tmp_pointer);
			}
			break;

			case INT_NUMBER:
			{
				int* tmp_pointer = (int*) _objects_to_write[callback_index].at(i);
				encoded_value = std::to_string(*tmp_pointer);
			}
			break;

			case STRING:
			{
				std::string* tmp_pointer = (std::string*) _objects_to_write[callback_index].at(i);
				encoded_value = (*tmp_pointer);
			}
			break;

			case EIGEN_OBJECT:
			{
				double* tmp_pointer = (double*) _objects_to_write[callback_index].at(i);
				int nrows = _objects_to_write_sizes[callback_index].at(i).first;
				int ncols = _objects_to_write_sizes[callback_index].at(i).second;

				Eigen::MatrixXd tmp_matrix = Eigen::MatrixXd::Zero(nrows, ncols);
				for(int k=0 ; k<nrows ; k++)
				{
					for(int l=0 ; l<ncols ; l++)
					{
						tmp_matrix(k,l) = tmp_pointer[k + ncols*l];
					}
				}

				encoded_value = encodeEigenMatrixJSON(tmp_matrix);
			}
			break;
		}

		if(encoded_value != "")
		{
				write_key_value_pairs.push_back(make_pair(_keys_to_write[callback_index].at(i),encoded_value));
		}
	}

	pipeset(write_key_value_pairs);
}






static inline Eigen::MatrixXd decodeEigenMatrixWithDelimiters(const std::string& str,
	char col_delimiter, char row_delimiter, const std::string& delimiter_set,
	size_t idx_row_end = std::string::npos)
{
	// Count number of columns
	size_t num_cols = 0;
	size_t idx = 0;
	size_t idx_col_end = str.find_first_of(row_delimiter);
	while (idx < idx_col_end) {
		// Skip over extra whitespace
		idx = str.find_first_not_of(' ', idx);
		if (idx >= idx_col_end) break;

		// Find next delimiter
		idx = str.find_first_of(col_delimiter, idx + 1);
		++num_cols;
	}
	if (idx > idx_col_end) idx = idx_col_end;

	// Count number of rows
	size_t num_rows = 1;  // First row already traversed
	while (idx < idx_row_end) {
		// Skip over irrelevant characters
		idx = str.find_first_not_of(row_delimiter, idx);
		if (idx >= idx_row_end) break;

		// Find next delimiter
		idx = str.find_first_of(row_delimiter, idx + 1);
		++num_rows;
	}

	// Check number of rows and columns
	if (num_cols == 0)
		throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
	if (num_rows == 1) {
		// Convert to vector
		num_rows = num_cols;
		num_cols = 1;
	}

	// Parse matrix
	Eigen::MatrixXd matrix(num_rows, num_cols);
	std::string str_local(str);
	for (char delimiter : delimiter_set) {
		std::replace(str_local.begin(), str_local.end(), delimiter, ' ');
	}
	std::stringstream ss(str_local);
	for (size_t i = 0; i < num_rows; ++i) {
		for (size_t j = 0; j < num_cols; ++j) {
			std::string val;
			ss >> val;
			try {
				matrix(i,j) = std::stod(val);
			} catch (const std::exception& e) {
				throw std::runtime_error("RedisClient: Failed to decode Eigen Matrix from: " + str + ".");
			}
		}
	}

	return matrix;
}

Eigen::MatrixXd RedisClient::decodeEigenMatrixString(const std::string& str) {
	return decodeEigenMatrixWithDelimiters(str, ' ', ';', ";");
}

Eigen::MatrixXd RedisClient::decodeEigenMatrixJSON(const std::string& str) {
	// Find last nested row delimiter
	size_t idx_row_end = str.find_last_of(']');
	if (idx_row_end != std::string::npos) {
		size_t idx_temp = str.substr(0, idx_row_end).find_last_of(']');
		if (idx_temp != std::string::npos) idx_row_end = idx_temp;
	}
	return decodeEigenMatrixWithDelimiters(str, ',', ']', ",[]", idx_row_end);
}

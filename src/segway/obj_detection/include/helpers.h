#include <memory>
#include <string>
#include <stdexcept>


template<typename ... Args> 
std::string string_format( const std::string& format, Args ... args );
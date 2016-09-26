// #ifndef STRING_OPERATIONS_H
// #define STRING_OPERATIONS_H

// #include <string>
// #include <sstream>

// class StringOperations {
// 	//Two utilities functions to split a stirng
// 	static std::vector<std::string> & stringSplitElems(const std::string &s, char delim, std::vector<std::string> &elems) {
// 	    std::stringstream ss(s);
// 	    std::string item;
// 	    while (std::getline(ss, item, delim)) {
// 	        elems.push_back(item);
// 	    }
// 	    return elems;
// 	}

// 	static std::vector<std::string> stringSplit(const std::string &s, char delim) {
// 	    std::vector<std::string> elems;
// 	    stringSplitElems(s, delim, elems);
// 	    return elems;
// 	}

// };



// #endif
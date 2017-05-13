/* Revised by: Zongyao Jin
 *
 * Util.h  Created on: 11 May 2014
 * Copyright (c) 2014 Derek Molloy (www.derekmolloy.ie)
 */

#ifndef UTIL_H_
#define UTIL_H_
#include<string>
using std::string;

namespace ZJ {


int write(string path, string filename, string value);
int write(string path, string filename, int value);
string read(string path, string filename);


} /* namespace ZJ */

#endif /* UTIL_H_ */

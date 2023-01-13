//
// Created by 10533 on 2023/1/11.
//

#include <iostream>

int main(int argc, char* argv[])
{
    if(argc==1)
	{
		std::cout<<"no input\n";
		return 0;
	}
	Parser parser;
	Scene scene = parser.get_scene(argv[1]);
    return 0;
}
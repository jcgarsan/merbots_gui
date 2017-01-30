/*
 * reading_from_string.cpp
 *
 *  Created on: 24/4/2015
 *      Author: angeld
 */




#include "io_data_file/Jzon.h"

#include <iostream>

int main()
{
        std::string jsonData = "{\"name\":\"John\",\"age\":20}";

        Jzon::Object rootNode;
        Jzon::Parser parser(rootNode, jsonData);
        if (!parser.Parse())
        {
                std::cout << "Error: " << parser.GetError() << std::endl;
        }
        else
        {
                std::cout << "Name: " << rootNode.Get("name").ToString() << std::endl;
                std::cout << "Age: " << rootNode.Get("age").ToInt() << std::endl;
        }

        return 0;
}

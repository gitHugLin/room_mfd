//
// Created by 林奇 on 16/7/31.
//

#include "../include/MyAlgorithm.h"



MyAlgorithm::MyAlgorithm()
{
}

MyAlgorithm::~MyAlgorithm()
{
}

void MyAlgorithm::save(const String& filename) const
{
    FileStorage fs(filename, FileStorage::WRITE);
    fs << getDefaultName() << "{";
    fs << "format" << (int)3;
    write(fs);
    fs << "}";
}

String MyAlgorithm::getDefaultName() const
{
    return String("my_object");
}
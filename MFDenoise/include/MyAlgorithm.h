//
// Created by 林奇 on 16/7/31.
//

#ifndef MFD_MYALGORITHM_H
#define MFD_MYALGORITHM_H

#include "iostream"
#include "vector"
#include "string"
#include "opencv2/opencv.hpp"
#include "MyOcl.hpp"

using namespace cv;
using namespace std;

class MyAlgorithm
{
public:
    MyAlgorithm();
    virtual ~MyAlgorithm();

    virtual void clear() {}

    virtual void write(FileStorage& fs) const { (void)fs; }

    virtual void read(const FileNode& fn) { (void)fn; }

    virtual bool empty() const { return false; }

    template<typename _Tp> static Ptr<_Tp> read(const FileNode& fn)
    {
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }
    template<typename _Tp> static Ptr<_Tp> load(const String& filename, const String& objname=String())
    {
        FileStorage fs(filename, FileStorage::READ);
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }
    template<typename _Tp> static Ptr<_Tp> loadFromString(const String& strModel, const String& objname=String())
    {
        FileStorage fs(strModel, FileStorage::READ + FileStorage::MEMORY);
        FileNode fn = objname.empty() ? fs.getFirstTopLevelNode() : fs[objname];
        Ptr<_Tp> obj = _Tp::create();
        obj->read(fn);
        return !obj->empty() ? obj : Ptr<_Tp>();
    }

    virtual void save(const String& filename) const;

    virtual String getDefaultName() const;
};


#endif //MFD_MYALGORITHM_H

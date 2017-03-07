//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//
//  Copyright (c) 2017 by
//       __      _     _         _____
//    /\ \ \__ _| |__ (_) __ _  /__   \_ __ _   _  ___  _ __   __ _
//   /  \/ / _` | '_ \| |/ _` |   / /\/ '__| | | |/ _ \| '_ \ / _` |
//  / /\  / (_| | | | | | (_| |  / /  | |  | |_| | (_) | | | | (_| |
//  \_\ \/ \__, |_| |_|_|\__,_|  \/   |_|   \__,_|\___/|_| |_|\__, |
//         |___/                                              |___/
//
//  <nghiatruong.vn@gmail.com>
//  All rights reserved.
//
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+

#pragma once

#include "SPlisHSPlasH/Common.h"

#include <string>
#include <vector>
#include <future>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <cassert>

using namespace SPH;
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// DataBuffer class
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class DataBuffer
{
public:
    DataBuffer(size_t bufferSize = 0);

    ~DataBuffer();

    size_t size() const
    {
        return static_cast<size_t>(m_Buffer.size());
    }

    void resize(size_t bufferSize)
    {
        m_Buffer.resize(bufferSize);
    }

    void reserve(size_t bufferSize)
    {
        m_Buffer.reserve(bufferSize);
    }

    const unsigned char* data() const
    {
        return m_Buffer.data();
    }

    std::vector<unsigned char>& buffer()
    {
        return m_Buffer;
    }

    void clearBuffer()
    {
        m_Buffer.resize(0);
    }

    //////////////////////////////////////////////////////////////////////
    /// replace data in the buffer with new data
    //////////////////////////////////////////////////////////////////////
    void set_data(const DataBuffer& dataBuffer);
    void set_data(const unsigned char* arrData, size_t dataSize);

    template<class T>
    void set_to_float(T data)
    {
        clearBuffer();
        push_back<float>(static_cast<float>(data));
    }

    template<class T>
    void set_to_double(T data)
    {
        clearBuffer();
        push_back<double>(static_cast<double>(data));
    }

    template<class T>
    void set_data(T data)
    {
        clearBuffer();
        push_back<T>(data);
    }

    // need this, it is similar to the one above, but using reference parameter
    template<class T>
    void set_data(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        clearBuffer();
        push_back<T>(vData, bWriteVectorSize);
    }

    template<class T>
    void set_to_float_array(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        clearBuffer();
        push_back_to_float_array<T>(vData, bWriteVectorSize);
    }

    template<class T>
    void set_to_double_array(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        clearBuffer();
        push_back_to_double_array<T>(vData, bWriteVectorSize);
    }


    //////////////////////////////////////////////////////////////////////
    /// append data
    //////////////////////////////////////////////////////////////////////
    void push_back(const DataBuffer& dataBuffer);
    void push_back(const unsigned char* arrData, size_t dataSize);

    template<class T>
    void push_back(T value)
    {
        size_t dataSize = sizeof(T);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        T Tdata = value;
        memcpy((void*) & (m_Buffer.data()[endOffset]), (void*)&Tdata, dataSize);
    }

    template<class T>
    void push_back(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        // any vector data begins with the number of vector elements
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(T);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        memcpy((void*) & (m_Buffer.data()[endOffset]), (void*)vData.data(), dataSize);
    }


    void push_back(const std::vector<Vector3r>& vData, bool bWriteVectorSize = true)
    {
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(Real) * 3;
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        Real* buff_ptr = (Real*) & (m_Buffer.data()[endOffset]);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            const Vector3r& vec  = vData[i];
            buff_ptr[i * 3]     = vec[0];
            buff_ptr[i * 3 + 1] = vec[1];
            buff_ptr[i * 3 + 2] = vec[2];
        }
    }

    template<class T>
    void push_back(const std::vector<std::vector<T> >& vData)
    {
        const unsigned int numElements = (unsigned int)vData.size();
        push_back(numElements);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            const std::vector<T>& vec = vData[i];
            push_back(vec);
        }
    }

    template<class T>
    void push_back_to_float(T value)
    {
        size_t dataSize = sizeof(float);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        float fldata = static_cast<float>(value);
        memcpy((void*) & (m_Buffer.data()[endOffset]), (void*)&fldata, dataSize);

    }

    template<class T>
    void push_back_to_float_array(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(float);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        float* buff_ptr = (float*) & (m_Buffer.data()[endOffset]);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            buff_ptr[i] = static_cast<float>(vData[i]);
        }

    }


    void push_back_to_float_array(const std::vector<Vector3r>& vData, bool bWriteVectorSize = true)
    {
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(float) * 3;
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        float* buff_ptr = (float*) & (m_Buffer.data()[endOffset]);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            const Vector3r& vec = vData[i];
            buff_ptr[i * 3]     = static_cast<float>(vec[0]);
            buff_ptr[i * 3 + 1] = static_cast<float>(vec[1]);
            buff_ptr[i * 3 + 2] = static_cast<float>(vec[2]);
        }
    }


    template<class T>
    void push_back_to_double(T value)
    {
        size_t dataSize = sizeof(double);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        double dbdata = (double)value;
        memcpy((void*) & (m_Buffer.data()[endOffset]), (void*)&dbdata, dataSize);
    }

    template<class T>
    void push_back_to_double_array(const std::vector<T>& vData, bool bWriteVectorSize = true)
    {
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(double);
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        double* buff_ptr = (double*) & (m_Buffer.data()[endOffset]);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            buff_ptr[i] = static_cast<double>(vData[i]);
        }
    }


    void push_back_to_double_array(const std::vector<Vector3r>& vData, bool bWriteVectorSize = true)
    {
        if(bWriteVectorSize)
        {
            const unsigned int numElements = (unsigned int)vData.size();
            push_back(numElements);
        }

        size_t dataSize = vData.size() * sizeof(double) * 3;
        size_t endOffset = m_Buffer.size();
        resize(endOffset + dataSize);

        double* buff_ptr = (double*) & (m_Buffer.data()[endOffset]);

        for(size_t i = 0; i < vData.size(); ++i)
        {
            const Vector3r& vec = vData[i];
            buff_ptr[i * 3]     = static_cast<double>(vec[0]);
            buff_ptr[i * 3 + 1] = static_cast<double>(vec[1]);
            buff_ptr[i * 3 + 2] = static_cast<double>(vec[2]);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// get data
    ////////////////////////////////////////////////////////////////////////////////
    template<class T>
    size_t get_data(T& value, size_t startOffset = 0)
    {
        size_t dataSize = (size_t)sizeof(T);
        assert(startOffset + dataSize <= m_Buffer.size());

        memcpy(&value, &m_Buffer.data()[startOffset], dataSize);

        return dataSize;
    }

    template<class T>
    size_t get_data(std::vector<T>& vData, size_t startOffset = 0, unsigned int vSize = 0)
    {
        size_t segmentStart = startOffset;
        size_t segmentSize  = 0;
        unsigned int numElements  = vSize;

        if(numElements == 0)
        {
            segmentSize = (size_t)sizeof(unsigned int);
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy(&numElements, &m_Buffer.data()[segmentStart], segmentSize);
            segmentStart += segmentSize;
        }

        vData.resize(numElements);

        segmentSize = (size_t)(sizeof(T) * numElements);
        assert(segmentStart + segmentSize <= m_Buffer.size());

        memcpy(vData.data(), &m_Buffer.data()[segmentStart], segmentSize);
        segmentStart += segmentSize;

        return (segmentStart - startOffset);
    }



    template<class T>
    size_t get_data(std::vector<Vector3r>& vData, size_t startOffset = 0, unsigned int vSize = 0)
    {
        size_t segmentStart = startOffset;
        size_t segmentSize  = 0;
        unsigned int numElements  = vSize;

        if(numElements == 0)
        {
            segmentSize = (size_t)sizeof(unsigned int);
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy(&numElements, &m_Buffer.data()[segmentStart], segmentSize);
            segmentStart += segmentSize;
        }

        vData.resize(numElements);

        segmentSize = (size_t)(3 * sizeof(T) * numElements);
        assert(segmentStart + segmentSize <= m_Buffer.size());

        T* buff_ptr = (T*)&m_Buffer.data()[segmentStart];

        for(size_t i = 0; i < numElements; ++i)
        {
            vData[i] = Vector3r(static_cast<Real>(buff_ptr[i * 3]), static_cast<Real>(buff_ptr[i * 3 + 1]), static_cast<Real>(buff_ptr[i * 3 + 2]));
        }

        segmentStart += segmentSize;

        return (segmentStart - startOffset);
    }


    template<class T, int N>
    size_t get_float_array(std::vector<float>& vData, size_t startOffset = 0, unsigned int vSize = 0)
    {
        size_t segmentStart = startOffset;
        size_t segmentSize  = 0;
        unsigned int numElements  = vSize;

        if(numElements == 0)
        {
            segmentSize = (size_t)sizeof(unsigned int);
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy(&numElements, &m_Buffer.data()[segmentStart], segmentSize);
            segmentStart += segmentSize;
        }

        vData.resize(numElements * N);

        T value;
        segmentSize = (size_t)(sizeof(T));

        for(size_t i = 0; i < vData.size(); ++i)
        {
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy((void*)(&value), &(m_Buffer.data()[segmentStart]), segmentSize);
            vData[i] = static_cast<float>(value);
            segmentStart += segmentSize;
        }

        return (segmentStart - startOffset);
    }


    template<class T, int N>
    size_t get_double_array(std::vector<double>& vData, size_t startOffset = 0, unsigned int vSize = 0)
    {
        size_t segmentStart = startOffset;
        size_t segmentSize  = 0;
        unsigned int numElements  = vSize;

        if(numElements == 0)
        {
            segmentSize = (size_t)sizeof(unsigned int);
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy(&numElements, &m_Buffer.data()[segmentStart], segmentSize);
            segmentStart += segmentSize;
        }

        vData.resize(numElements * N);

        T value;
        segmentSize = (size_t)(sizeof(T));

        for(size_t i = 0; i < vData.size(); ++i)
        {
            assert(segmentStart + segmentSize <= m_Buffer.size());

            memcpy((void*)(&value), &(m_Buffer.data()[segmentStart]), segmentSize);
            vData[i] = static_cast<double>(value);
            segmentStart += segmentSize;
        }

        return (segmentStart - startOffset);
    }


    template<class T>
    size_t get_data(std::vector<std::vector<T> >& vData, size_t startOffset = 0)
    {
        size_t segmentStart = startOffset;
        size_t segmentSize = (size_t)sizeof(unsigned int);
        assert(segmentStart + segmentSize <= m_Buffer.size());

        unsigned int numElements;
        memcpy(&numElements, &m_Buffer.data()[segmentStart], segmentSize);
        segmentStart += segmentSize;

        vData.resize(numElements);

        for(unsigned int i = 0; i < numElements; ++i)
        {
            segmentStart += get_data(vData[i], segmentStart);
        }

        return (segmentStart - startOffset);
    }

    size_t get_data(unsigned char* arrData, size_t dataSize, size_t startOffset = 0)
    {
        assert(startOffset + dataSize <= m_Buffer.size());
        memcpy(arrData, &m_Buffer.data()[startOffset], dataSize);

        return dataSize;
    }

private:
    size_t                     m_BufferSize;
    std::vector<unsigned char> m_Buffer;
};


//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
// DataIO class
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
//-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
class DataIO
{
public:
    DataIO(std::string dataRootFolder, std::string dataFolder,
           std::string fileName, std::string fileExtension);
    virtual ~DataIO();

    void reset_buffer();
    void flush_buffer(int fileID);
    void flush_buffer_async(int fileID);
    std::string get_file_name(int fileID);
    DataBuffer& getBuffer();

private:
    void create_output_folders();

    ////////////////////////////////////////////////////////////////////////////////
    bool               m_isOutputFolderCreated;
    std::string        m_DataRootFolder;
    std::string        m_DataFolder;
    std::string        m_FileName;
    std::string        m_FileExtension;
    DataBuffer         m_FileBuffer;
    std::future<void>  m_WriteFutureObj;

};
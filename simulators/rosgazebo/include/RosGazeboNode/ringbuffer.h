
/*
 *  This is an incomplete ring buffer template class;
 *  The idea is to be able to easily create
 *  ringbuffers for incoming points that are
 *  efficiently allocated and destroyed.
 *
 *  It's nowhere near done.
 *
 *  It's very similar to an stl vector. I'd use an
 *  stl vector if there were an easy way to force
 *  allocation of a large block of items, to not
 *  initialize those items, and to get the block
 *  for use in calls to opengl.  Alas, there isn't.
 *
 *  This one doesn't resize, and is genearlly much
 *  less powerful than a vector.  You have to want
 *  to use this to use it.
 *
 */

#ifndef _ringbuffer_h_
#define _ringbuffer_h_

#define CHECK

template <class nodeType>
class ringBuffer
{
public:
    ringBuffer(int size=0) {
        buffer=NULL;
        release();
        if (size)
            allocate(size);
    };

    ~ringBuffer() {
        release();
    };

    void release() {
        if (buffer)
            free(buffer);
        buffer=NULL;
        max_size=0;
        length=0;
        end_index=0;
    };

    void allocate(int size) {
        buffer=(nodeType *)malloc(sizeof(nodeType)*size);
        max_size=size;
        length=0;
        end_index=0;
    };
    inline void add(nodeType value) {
#ifdef CHECK
        if (end_index>=max_size)
            end_index=0;
#endif
        buffer[end_index]=value;
        end_index++;
#ifdef CHECK
        if (length<max_size)
            length++;
#endif
    };

    inline nodeType *back() {
        if (length > 0) return &(buffer[end_index-1]);
        else            return NULL;
    };
    nodeType *buffer;
    int max_size;
    int length;
    int end_index;
};


#endif



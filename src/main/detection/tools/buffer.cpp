// #include "buffer.hpp"

// template <typename Type>
// Buffer<Type>::Buffer()
// {
//     for (int i = 0; i < buffer_size; i++) {
//         buffer_status_[i] = EMPTY;
//     }
//     buffer_ = new Type[buffer_size];
// }

// template <typename Type>
// Buffer<Type>::~Buffer()
// {
//     delete buffer_;
// }

// template <typename Type>
// bool Buffer<Type>::empty()
// {
//     for (int i = 0; i < buffer_size; i++) {
//         if (buffer_status_[i] == FREE) {
//             return false;
//         }
//     }
//     return true;
// }

// template <typename Type>
// void Buffer<Type>::write(Type& value)
// {
//     for (int i = 0; i < buffer_size; i++) {
//         if (buffer_status_[i] != READING) {

//             buffer_[i]
//         }
//     }
// }


// template <typename Type>
// Type Buffer<Type>::read()
// {
    
// }


// template <typename Type>
// void Buffer<Type>::end_read()
// {
    
// }
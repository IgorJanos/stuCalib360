#ifndef UTILS_H
#define UTILS_H



template<class T, typename... Args> QSharedPointer<T> makeNew(Args... args) {
    return QSharedPointer<T>(new T(args...));
}



#endif // UTILS_H

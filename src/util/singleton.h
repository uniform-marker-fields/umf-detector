#ifndef __UMF_SINGLETON_H
#define __UMF_SINGLETON_H

#ifndef UMF
#if defined(__WIN32__) || defined(_WIN32)
#define UMF __declspec( dllexport )
#else
#define UMF
#endif
#endif

#include <cstddef>

namespace umf
{

template <class T>
class Singleton
{
public:
    static T* Instance() {
        if(!m_pInstance) m_pInstance = new T;
        assert(m_pInstance != NULL);
        return m_pInstance;
    }

	static void Release() {
		if(m_pInstance) delete m_pInstance;
        m_pInstance = NULL;
	}
protected:
    Singleton();
    ~Singleton();
private:
    Singleton(Singleton const&);
    Singleton& operator=(Singleton const&);
    UMF static T* m_pInstance;
};

//template <class T> T* Singleton<T>::m_pInstance = nullptr;

}

#endif // SINGLETON_H

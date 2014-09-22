#include <boost/thread.hpp>
#include <list>

#ifndef CONCURRENT_QUEUE_HPP
#define CONCURRENT_QUEUE_HPP

namespace amcl6d_tools
{

template<class T>
class concurrent_queue
{
    public:
        void push_back(T element)
        {
            boost::unique_lock<boost::shared_mutex> lock(m_mutex);
            m_list.push_back(element);
        }

        T pop_front()
        {
            boost::shared_lock<boost::shared_mutex> lock(m_mutex);
            if(m_list.empty())
            {
                return NULL;
            }
            else
            {
                boost::upgrade_lock<boost::shared_mutex> lock(m_mutex);
                T elem = m_list.front();
                m_list.pop_front();
                return elem;
            }
                
        }

        int size()
        {
            boost::shared_lock<boost::shared_mutex> lock(m_mutex);
            return m_list.size();
        }

        bool empty()
        {
            boost::shared_lock<boost::shared_mutex> lock(m_mutex);
            return m_list.empty();
        }

    private:
        std::list<T> m_list;

        boost::shared_mutex m_mutex;

};

}
#endif

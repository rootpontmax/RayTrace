////////////////////////////////////////////////////////////////////////////////////////////////////
// Simple class for thread pool.																  //
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <future>

////////////////////////////////////////////////////////////////////////////////////////////////////
class CThreadPool
{
public:
    CThreadPool( const size_t threadCount ) :
        m_bIsStopped( false )
    {
        m_threadPool.reserve( threadCount );
        for( size_t i = 0; i < threadCount; ++i )
        {
            m_threadPool.emplace_back([this]()
                {
                    for( ; ; )
                    {
                        std::function< void() > task;
                        if( !m_taskQueue.empty() || m_bIsStopped )
                        {
                            std::unique_lock< std::mutex > guard( m_queueMutex );
                            if( m_bIsStopped && m_taskQueue.empty() )
                                return;
                            else if( m_taskQueue.empty() )
                                continue;
                            
                            task = std::move( m_taskQueue.front() );
                            m_taskQueue.pop();
                        }
                        if( task )
                            task();
                    }
                }
            );
        }
        
    }
    /*
    template< class F, class... Args >
    auto void Add( F&& f, Args&&... args )->std::future< typename std::result_of<F(Args... )>::type>
    {
        std::unique_lock< std::mutex > guard( m_queueMutex );
    }
    
    //*/
    
private:
    std::vector< std::thread >              m_threadPool;
    std::queue< std::function< void() > >   m_taskQueue;
    std::mutex                              m_queueMutex;
    bool                                    m_bIsStopped;
};
////////////////////////////////////////////////////////////////////////////////////////////////////

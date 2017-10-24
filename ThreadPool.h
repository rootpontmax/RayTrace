////////////////////////////////////////////////////////////////////////////////////////////////////
// Simple class for thread pool.																  //
////////////////////////////////////////////////////////////////////////////////////////////////////
#pragma once
#include <vector>
#include <queue>
#include <thread>
#include <future>
#include <condition_variable>
#include <cassert>

////////////////////////////////////////////////////////////////////////////////////////////////////
class CThreadPool
{
public:
    CThreadPool( const size_t threadCount ) :
        m_balance( 0 ),
        m_bIsStopped( false )
    {
        auto threadFunc = [this]( const size_t threadID )
        {
            for( ; ; )
            {
                std::function< void( const size_t threadID ) > task;
                {
                    std::unique_lock< std::mutex > guard( m_queueMutex );
                            
                    m_condVar.wait( guard, [this]()
                    {
                        return ( !m_taskQueue.empty() || m_bIsStopped );
                    });
                            
                    if( m_bIsStopped || m_taskQueue.empty() )
                        return;
                            
                    task = std::move( m_taskQueue.front() );
                    m_taskQueue.pop();
                }
                task( threadID );
                        
                std::unique_lock< std::mutex > guard( m_doneMutex );
                --m_balance;
                m_condVarDone.notify_one();
            }
        };
        
        m_threadPool.reserve( threadCount );
        for( size_t i = 0; i < threadCount; ++i )
            m_threadPool.emplace_back( threadFunc, i );
    }
    
    ~CThreadPool()
    {
        m_bIsStopped = true;
        m_condVar.notify_all();
        for( size_t i = 0; i < m_threadPool.size(); ++i )
            m_threadPool[i].join();
    }
    
    template< typename F, typename... Args >
    auto Add( F&& f, Args&&... args )  ->std::future< decltype( f( 0, args... ) ) >
    {
        auto task = std::make_shared< std::packaged_task< decltype(f(0, args...))(const size_t)>>(
                std::bind(std::forward<F>(f), std::placeholders::_1, std::forward<Args>(args)...));
        auto res = task->get_future();
        {
            std::unique_lock< std::mutex > guard( m_queueMutex );
            assert( !m_bIsStopped );
            m_taskQueue.emplace( [task]( const size_t threadID ) { (*task)( threadID ); } );
            ++m_balance;
        }
        m_condVar.notify_one();        
        return res;
    }
    
    void WaitAllDone()
    {
        std::unique_lock< std::mutex > guard( m_doneMutex );
        m_condVarDone.wait( guard, [this]()
        {
            return ( 0 == m_balance );
        });
    }
    
    
private:
    std::vector< std::thread >              m_threadPool;
    std::queue< std::function< void( const size_t threadID ) > >   m_taskQueue;
    std::mutex                              m_queueMutex;
    std::condition_variable                 m_condVar;
    std::mutex                              m_doneMutex;
    std::condition_variable                 m_condVarDone;
    std::atomic<int>                        m_balance;
    std::atomic<bool>                       m_bIsStopped;
};
////////////////////////////////////////////////////////////////////////////////////////////////////

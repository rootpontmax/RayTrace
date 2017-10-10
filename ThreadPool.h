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
        m_threadPool.reserve( threadCount );
        for( size_t i = 0; i < threadCount; ++i )
        {
            m_threadPool.emplace_back([this]()
                {
                    for( ; ; )
                    {
                        std::function< void() > task;
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
                        task();
                        
                        std::unique_lock< std::mutex > guard( m_doneMutex );
                        --m_balance;
                        m_condVarDone.notify_one();
                    }
                }
            );
        }
    }
    
    ~CThreadPool()
    {
        m_bIsStopped = true;
        m_condVar.notify_all();
        for( size_t i = 0; i < m_threadPool.size(); ++i )
            m_threadPool[i].join();
    }
    
    template< class F, class... Args >
    //auto void Add( F&& f, Args&&... args ) -> std::future< typename std::result_of<F(Args... )>::type>
    void Add( F&& f, Args&&... args )
    {
        {
            using return_type = typename std::result_of< F(Args...) >::type;
            auto task = std::make_shared< std::packaged_task< return_type() > >( [f, args...]() { return f(args...); } ); 
            std::unique_lock< std::mutex > guard( m_queueMutex );
            assert( !m_bIsStopped );
            m_taskQueue.emplace( [task]() { (*task)(); } );
            ++m_balance;
        }
        m_condVar.notify_one();
    }
    
    void WaitAllDoneCRAP()
    {
        while( 0 != m_balance );
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
    std::queue< std::function< void() > >   m_taskQueue;
    std::mutex                              m_queueMutex;
    std::condition_variable                 m_condVar;
    std::mutex                              m_doneMutex;
    std::condition_variable                 m_condVarDone;
    std::atomic<int>                        m_balance;
    std::atomic<bool>                       m_bIsStopped;
};
////////////////////////////////////////////////////////////////////////////////////////////////////

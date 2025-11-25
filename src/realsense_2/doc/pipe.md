# {pipe} 组件的设计与使用

# 设计目的

为了简单地创建“生产者->消费者链条”，提高并行化程度。

## 设计一：

### 基本设计

定义：执行生产/消费函数的单元为 **节点**，通过 `SIMO` 类实现
将所有**节点**分为三类，源头生产者节点，终点消费者节点，中游节点（既是生产者，又是消费者）。前两者定义了特殊的模版名称 `SIMO_START` 与 `SIMO_END`。
源头生产者节点与中游生产者节点拥有自己的“产品存放”的地方，这里的实现是一组线程安全的队列，即`oqueue`，其是一个 `array<queue_mt>`，理论上来说，无论是有锁还是无锁都是没有关系的，但是由于是面向对于按照时间运行的任务，比如100HZ的摄像头设备读取，我们希望是其先产生的画面会被先处理，而后产生的画面会被后处理，所以选择自己实现了一个有锁队列，备选的有 `concurrentqueue` 详见 github。

```cpp
    auto producer = SIMO_START<int>(5u);
    auto consumer1 = SIMO_END<int, 3u>(producer.oqueues[0],
        [](auto &self, int products)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    );
```

### 额外考虑：

每个节点 hold 一个线程，还需要能够hold多个线程，节点可以生产到多个oqueue中，使得多个不同消费者可以使用，此即 `Nthreads` 与 `Npubs`
上面的 `SIMO_END` 启动了3个线程。

## 设计二：

需要能够手动暂停与感知暂停，以及退出与感知退出这样的线程管理功能，于是采用了 `SingalPub` 类，创建对象时使用对应的节点（可以是多个节点）初始化构造，然后就可以对那些节点进行启动，停止的操作了。

```cpp
auto sig_pub = SignalPub(consumer1);
sig_pub.pub(ThreadSignal::START);
```



## full example


```cpp
auto producer = SIMO_START<int>(5u);
    auto consumer1 = SIMO_END<int, 3u>(producer.oqueues[0],
        [](auto &self, int products)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    );
    auto sig_pub = SignalPub(consumer1);
    sig_pub.pub(ThreadSignal::START);
    std::thread([&producer]{
        int i = 0;
        while(true)
        {
            producer.push(i);
            i++;
            std::this_thread::sleep_for(std::chrono::microseconds(333));
        }
    }).detach();
    char ctrl;
    while(true)
    {
        scanf("%c", &ctrl);
        if(ctrl == 'p')
            sig_pub.pub(ThreadSignal::STOP);
        else if(ctrl == 's')
            sig_pub.pub(ThreadSignal::START);    
        else if(ctrl == 'e')
            sig_pub.pub(ThreadSignal::EXIT);
        printf("%d\n", consumer1.threads_num());
    }
```
<center>Optimize msg Transmission Delay</center>

# MultiTopicPublisher and MultiTopicSubscriber in ROS 2

This project consists of two ROS 2 nodes: `MultiTopicPublisher` and `MultiTopicSubscriber`. The publisher node publishes messages of varying sizes on different topics, and the subscriber node subscribes to these topics and logs the message delays.

## Prerequisites

- ROS 2 (Foxy, Galactic, Humble, or Rolling)
- A C++ compiler

## Response



# Description:

## **Default Mode**

- At the default mode, we are able to see the magnitude of message and corresponding message transmission delay intuitively. By and Large, the delay time with greater size is high, the message at 20 MB recieved delays about 27 ms. Furthermore,  it doesn't lose any packages during the transmission.
```
[INFO] [1721197216.257570265] [multi_topic_subscriber]: 186 Message Delay in topic_1M: 1.042 ms
[INFO] [1721197216.258839552] [multi_topic_subscriber]: 187 Message Delay in topic_2M: 1.598 ms
[INFO] [1721197216.264609408] [multi_topic_subscriber]: 188 Message Delay in topic_5M: 6.082 ms
[INFO] [1721197216.278670978] [multi_topic_subscriber]: 189 Message Delay in topic_10M: 15.079 ms
[INFO] [1721197216.303249270] [multi_topic_subscriber]: 190 Message Delay in topic_20M: 27.252 ms
```

## **Quality of Service in reliable**

- At the beginning of the code excution, the message transmission delay is exceedomgly high and unordered. The message size is at 5 MB only, but the transmission delay reachs 525.388ms. However, as the process continues, the transmision delay decrase and stabilizes.

```
[INFO] [1721202933.311354497] [multi_topic_subscriber]: 1 Message Delay in topic_1M: 13.812 ms
[INFO] [1721202933.325958039] [multi_topic_subscriber]: 2 Message Delay in topic_2M: 26.732 ms
[INFO] [1721202933.354645134] [multi_topic_subscriber]: 4 Message Delay in topic_10M: 41.292 ms
[INFO] [1721202933.409627629] [multi_topic_subscriber]: 5 Message Delay in topic_20M: 76.263 ms

[INFO] [1721202933.801555007] [multi_topic_subscriber]: 7 Message Delay in topic_2M: 2.783 ms
[INFO] [1721202933.826904624] [multi_topic_subscriber]: 9 Message Delay in topic_10M: 17.644 ms
[INFO] [1721202933.828388692] [multi_topic_subscriber]: 3 Message Delay in topic_5M: 525.388 ms
[INFO] [1721202933.829628714] [multi_topic_subscriber]: 8 Message Delay in topic_5M: 29.080 ms
[INFO] [1721202933.853174897] [multi_topic_subscriber]: 10 Message Delay in topic_20M: 34.149 ms
```

- After a few seconds, the message transmission delay reaches a stable condition. the delay time is proportional to the size of the transmitted message, ranging from 1 MB to 20 MB, with delay approximately from ms to 20 ms.

```
[INFO] [1721203006.661649328] [multi_topic_subscriber]: 391 Message Delay in topic_1M: 1.155 ms
[INFO] [1721203006.662787136] [multi_topic_subscriber]: 392 Message Delay in topic_2M: 1.676 ms
[INFO] [1721203006.666838437] [multi_topic_subscriber]: 393 Message Delay in topic_5M: 4.324 ms
[INFO] [1721203006.676291257] [multi_topic_subscriber]: 394 Message Delay in topic_10M: 10.536 ms
[INFO] [1721203006.693729794] [multi_topic_subscriber]: 395 Message Delay in topic_20M: 20.111 ms
```
- However, sometimes the situation differs. In such case, the relationship between delay and message size exponential. From 1 to 5 MB, the discrepancy is slight compared to the previous pattern. But as the message size grows, the differences become more pronounced, with over 30% delay at 10 MB and over 50% delay at 20 MB.

```
[INFO] [1721202974.163139377] [multi_topic_subscriber]: 67 Message Delay in topic_2M: 1.880 ms
[INFO] [1721202974.170031269] [multi_topic_subscriber]: 68 Message Delay in topic_5M: 7.088 ms
[INFO] [1721202974.171809505] [multi_topic_subscriber]: 66 Message Delay in topic_1M: 11.278 ms
[INFO] [1721202974.182974735] [multi_topic_subscriber]: 69 Message Delay in topic_10M: 13.997 ms
[INFO] [1721202974.207501570] [multi_topic_subscriber]: 70 Message Delay in topic_20M: 27.628 ms
```

### Improving method 

- The QoS (quality of service) in reliable mode is utilized. In this mode, communication between the publisher and the subscriber uses a mechanism that ensures message delivery. If the message is not received successfully on the first delivery, the system automatically retries until the message is received successfully or the communication fails completely (such as a timeout). From above printed log of beginning excution code, the packages recieved are not in sequence. In default mode, those packages will lose, because they won't send in second time. However, for the QoS in reliable mode, it will ensure every package is sent and recieved successfully.

```
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
            qos.reliable();  // Use reliable QoS for better performance
```

## **Quality of Service in best_effort**

- Compared to the delay time in default mode, we can see the optimizing of transmission delay is limited, or is small enough to ignore the improvement. As the size of message is small (less than 5 MB), the delay actually is similar. Well, increasing magnitude to 10 MB and 20 MB, the transmission delay decrases aproximately 1.5 ms. 

```
[INFO] [1721206138.849049251] [multi_topic_subscriber]: 31 Message Delay in topic_1M: 0.990 ms
[INFO] [1721206138.850857769] [multi_topic_subscriber]: 32 Message Delay in topic_2M: 2.152 ms
[INFO] [1721206138.857454101] [multi_topic_subscriber]: 33 Message Delay in topic_5M: 6.843 ms
[INFO] [1721206138.869618663] [multi_topic_subscriber]: 34 Message Delay in topic_10M: 13.063 ms
[INFO] [1721206138.893359037] [multi_topic_subscriber]: 35 Message Delay in topic_20M: 26.396 ms
```
                                                                                                                                  
### Improving method 

- The QoS (quality of service) in best_effort mode is utilized. Best-Effort is the simplest QoS service model. Users can send any number of packets at any time without informing the network. During the Best-Effort service, the network sends packets as much as possible, but does not guarantee performance such as delay and packet loss rate. The Best-Effort service model is the default service model of the Internet, applicable to services that have low performance requirements such as delay and packet loss rate. Therefore, at the commence of code excution, the first several packages do always disappear.

```
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
            qos.best_effort();  // Use reliable QoS for better performance
```


## **Threading mode**

Similar to other modes, at the beginning of code excution, the unordered packages and the fairly high transmission delay frequently occur.
```
[INFO] [1721270272.223784823] [multi_topic_subscriber]: 1 Message Delay in topic_2M: 18.796 ms
[INFO] [1721270272.233387736] [multi_topic_subscriber]: 1 Message Delay in topic_1M: 28.613 ms
[INFO] [1721270272.271880754] [multi_topic_subscriber]: 3 Message Delay in topic_10M: 66.389 ms
[INFO] [1721270272.290359267] [multi_topic_subscriber]: 2 Message Delay in topic_5M: 85.116 ms
[INFO] [1721270272.310114736] [multi_topic_subscriber]: 3 Message Delay in topic_20M: 104.444 ms
[INFO] [1721270272.709670621] [multi_topic_subscriber]: 6 Message Delay in topic_1M: 1.550 ms
[INFO] [1721270272.714193637] [multi_topic_subscriber]: 7 Message Delay in topic_2M: 2.878 ms
[INFO] [1721270272.726952557] [multi_topic_subscriber]: 8 Message Delay in topic_5M: 7.433 ms
[INFO] [1721270272.749728417] [multi_topic_subscriber]: 9 Message Delay in topic_10M: 18.932 ms
[INFO] [1721270272.788460149] [multi_topic_subscriber]: 10 Message Delay in topic_20M: 37.164 ms
```

- This is the most ideal condition after a few seconds. In such case, the message transmission delay for all size of message shrinks significantly. As for the small messages, the transmission speed increses from 10 to 20 precentage. In regard to the larger message, the effect of optimization has marked advancement, which can reach over 45%. 
```
[INFO] [1721272432.411131388] [multi_topic_subscriber]: 51 Message Delay in topic_1M: 0.836 ms
[INFO] [1721272432.420382491] [multi_topic_subscriber]: 52 Message Delay in topic_2M: 1.468 ms
[INFO] [1721272432.454314090] [multi_topic_subscriber]: 53 Message Delay in topic_5M: 3.523 ms
[INFO] [1721272432.506053734] [multi_topic_subscriber]: 54 Message Delay in topic_10M: 7.779 ms
[INFO] [1721272432.600891836] [multi_topic_subscriber]: 55 Message Delay in topic_20M: 15.629 ms
```

- It is because mltuiple threads operate the shared recourse (e.g. in the announcement queue) that are likely to give rise to compete. In such case, some the message will get lose or be at unordered.
```
[INFO] [1721273884.466604839] [multi_topic_subscriber]: 231 Message Delay in topic_2M: 1.641 ms
[INFO] [1721273884.565072912] [multi_topic_subscriber]: 232 Message Delay in topic_5M: 6.905 ms
[INFO] [1721273884.570696148] [multi_topic_subscriber]: 232 Message Delay in topic_20M: 18.722 ms
[INFO] [1721273884.736710207] [multi_topic_subscriber]: 234 Message Delay in topic_10M: 7.470 ms
[INFO] [1721273884.927803652] [multi_topic_subscriber]: 235 Message Delay in topic_1M: 0.530 ms
```

### Improving method

- A thread is the smallest unit that an operating system can schedule, and a process can contain one or more threads, each sharing the process's resources. Multithreaded programming can excute multiple tasks in parallel in a single process, thus improving the excution efficiency of the program. Compared to the default mode, threading method has some advantages.It is able to excute mutli-tasks concurrently, and enhance the response, as well operation. Moreover, threading can share the memory and resource, thereby utlizing system resource in more efficient way.  

```
// Start a new thread for each publisher
threads_.emplace_back(std::thread(&MultiTopicPublisher::publish_loop, this, publisher, data_size));

    /**
     * @brief Destructor to clean up threads.
     */
    ~MultiTopicPublisher()
    {
        for (auto &thread : threads_) {
            if (thread.joinable()) {
                thread.join(); // Ensure all threads are properly joined
            }
        }
    }
```

- I also try to use the mutex to protect those resource from chaos, under the environment of shared resource operated by multiple threads. It is helpful, but not at all. As the sending time over 1100, the cases of chaos and missing package happen again.

```
std::lock_guard<std::mutex> lock(mutex_); // Protect shared resources with mutex 
```

## **CPU Affinity mode**

- As for the major of modes, their initial response time is quite slow, but CPU_Affinity doesn't need to worry about it. The longest waitting time does even not suspass 50 ms. Moreover, the missing package will be resent to the subscriper so no passage will get lose in this mode.

```
[INFO] [1721287862.079263132] [multi_topic_subscriber]: 2 Message Delay in topic_2M: 3.824 ms
[INFO] [1721287862.085980572] [multi_topic_subscriber]: 1 Message Delay in topic_1M: 12.374 ms
[INFO] [1721287862.089782354] [multi_topic_subscriber]: 3 Message Delay in topic_5M: 10.196 ms
[INFO] [1721287862.111804889] [multi_topic_subscriber]: 4 Message Delay in topic_10M: 21.342 ms
[INFO] [1721287862.160486340] [multi_topic_subscriber]: 5 Message Delay in topic_20M: 47.381 ms

[INFO] [1721287862.577018235] [multi_topic_subscriber]: 7 Message Delay in topic_2M: 2.038 ms
[INFO] [1721287862.584822133] [multi_topic_subscriber]: 8 Message Delay in topic_5M: 7.696 ms
[INFO] [1721287862.585481633] [multi_topic_subscriber]: 6 Message Delay in topic_1M: 11.863 ms
[INFO] [1721287862.596697943] [multi_topic_subscriber]: 9 Message Delay in topic_10M: 11.724 ms
[INFO] [1721287862.621980170] [multi_topic_subscriber]: 10 Message Delay in topic_20M: 25.128 ms

```

The case of chaos in sequence frequently occur in the CPU_Affinity. This may be due to the different data transmission and operation time for each topic. When it comes to optimization of delay, it makes works. However, I cannot figure out why sometimes this way is more helpful.
```
[INFO] [1721290427.297562795] [multi_topic_subscriber]: 72 Message Delay in topic_2M: 2.129 ms
[INFO] [1721290427.304664749] [multi_topic_subscriber]: 73 Message Delay in topic_5M: 7.001 ms
[INFO] [1721290427.306123528] [multi_topic_subscriber]: 71 Message Delay in topic_1M: 11.054 ms
[INFO] [1721290427.317564653] [multi_topic_subscriber]: 74 Message Delay in topic_10M: 12.747 ms
[INFO] [1721290427.343325262] [multi_topic_subscriber]: 75 Message Delay in topic_20M: 25.611 ms


[INFO] [1721290328.276797928] [multi_topic_subscriber]: 216 Message Delay in topic_1M: 0.851 ms
[INFO] [1721290328.278099507] [multi_topic_subscriber]: 217 Message Delay in topic_2M: 1.076 ms
[INFO] [1721290328.281931451] [multi_topic_subscriber]: 218 Message Delay in topic_5M: 3.746 ms
[INFO] [1721290328.290180265] [multi_topic_subscriber]: 219 Message Delay in topic_10M: 8.123 ms
[INFO] [1721290328.308363884] [multi_topic_subscriber]: 220 Message Delay in topic_20M: 18.089 ms


```

### Improving method

- CPU Affinity refers to the ability to associate a particular computing resource, such as a processor core, with a specific task or thread. It allows a system administrator or developer to bind a task or thread to a particular processor core, improving system performance and efficiency. The advantages of CPU affinity are: it improves the CPU cache hit ratio by reducing cache invalidation and data migration overhead; it reduces the cost of context switching by lowering the overhead of scheduling and switching processes or threads; and it enhances parallelism and throughput by preventing multiple tasks or threads from competing for the same processor core.

```
    // Bind node to specific CPU core
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    int rc = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
```

## **KeepLast(n) mode**

- The initail response speed of KeepLast(n) is fast as well. Even the transmission delay for the largest size of message doesn't exceed 50ms. Furthermore, no one package miss during the transmission.
```
[INFO] [1721379187.740378222] [multi_topic_subscriber]: Message Delay in topic_2M: 3.745 ms (Seq: 2)
[INFO] [1721379187.747197923] [multi_topic_subscriber]: Message Delay in topic_1M: 12.482 ms (Seq: 1)
[INFO] [1721379187.750363039] [multi_topic_subscriber]: Message Delay in topic_5M: 9.966 ms (Seq: 3)
[INFO] [1721379187.770893041] [multi_topic_subscriber]: Message Delay in topic_10M: 20.345 ms (Seq: 4)
[INFO] [1721379187.814093534] [multi_topic_subscriber]: Message Delay in topic_20M: 42.930 ms (Seq: 5)


[INFO] [1721379188.238197437] [multi_topic_subscriber]: Message Delay in topic_2M: 2.177 ms (Seq: 7)
[INFO] [1721379188.246208415] [multi_topic_subscriber]: Message Delay in topic_5M: 8.265 ms (Seq: 8)
[INFO] [1721379188.246824232] [multi_topic_subscriber]: Message Delay in topic_1M: 12.100 ms (Seq: 6)
[INFO] [1721379188.259150857] [multi_topic_subscriber]: Message Delay in topic_10M: 14.202 ms (Seq: 9)
[INFO] [1721379188.283462277] [multi_topic_subscriber]: Message Delay in topic_20M: 27.057 ms (Seq: 10)
```

- The message transmission delay for all size of message shrinks significantly. As for the small messages, the transmission speed increses from 10 to 20 precentage. In regard to the larger message, the effect of optimization has marked advancement, which can reach over 45%. 
```
[INFO] [1721380368.163296044] [multi_topic_subscriber]: Message Delay in topic_1M: 1.085 ms (Seq: 936)
[INFO] [1721380368.164514612] [multi_topic_subscriber]: Message Delay in topic_2M: 1.697 ms (Seq: 937)
[INFO] [1721380368.168578231] [multi_topic_subscriber]: Message Delay in topic_5M: 4.426 ms (Seq: 938)
[INFO] [1721380368.178124994] [multi_topic_subscriber]: Message Delay in topic_10M: 10.624 ms (Seq: 939)
[INFO] [1721380368.195626857] [multi_topic_subscriber]: Message Delay in topic_20M: 20.538 ms (Seq: 940)

```

### Improving method

The KeepLast(n) policy reduces memory usage by limiting the number of messages in the message queue. Only the latest n messages are retained, and the old messages are automatically discarded. This effectively controls the size of the message cache and avoids excessive memory consumption caused by message accumulation. Under high message frequency and unstable network conditions, the KeepLast(n) strategy can provide better performance. Because only the most recent messages are cached, the system can process those messages more quickly without having to process and store excessive amounts of older messages.
```
            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));  // Adjust QoS settings as needed
```

## **Zero Copy**

Compared by any other mode, the condition of "zero copy" from origin is best. No missing package and no unordered seqence. Moreover, even the transmission delay of it is fairly small. The average of them is only at about 2 ms no matter the size of package.

```
[INFO] [1721791827.619290474] [multi_topic_hbmem_subscriber]: Message 1 Delay in topic_1M: 2.226 ms
[INFO] [1721791827.628787143] [multi_topic_hbmem_subscriber]: Message 2 Delay in topic_2M: 2.188 ms
[INFO] [1721791827.638581911] [multi_topic_hbmem_subscriber]: Message 3 Delay in topic_5M: 2.188 ms
[INFO] [1721791827.647998744] [multi_topic_hbmem_subscriber]: Message 4 Delay in topic_10M: 2.352 ms
[INFO] [1721791827.657314266] [multi_topic_hbmem_subscriber]: Message 5 Delay in topic_20M: 2.126 ms
```

As the code gradually stable, the effect of "zero copy" is stronger. In such case, the size of message isn't the hinder for transmission speed any more. Therefore, the time delay is able to be shorten to above 0.5ms, which is impossible for any other mode used before.
```
[INFO] [1721791894.774972780] [multi_topic_hbmem_subscriber]: Message 156 Delay in topic_1M: 0.686 ms
[INFO] [1721791894.783624739] [multi_topic_hbmem_subscriber]: Message 157 Delay in topic_2M: 0.449 ms
[INFO] [1721791894.793177474] [multi_topic_hbmem_subscriber]: Message 158 Delay in topic_5M: 0.487 ms
[INFO] [1721791894.802381536] [multi_topic_hbmem_subscriber]: Message 159 Delay in topic_10M: 0.698 ms
[INFO] [1721791894.811270915] [multi_topic_hbmem_subscriber]: Message 160 Delay in topic_20M: 0.550 ms
```

### Improving Method

Zero Copy is an I/O operation optimization technique that quickly and efficiently moves data from a file system to a network interface without having to copy it from kernel space to user space.

In traditional I/O operations, data is often copied multiple times between kernel space and user space. For example, when data is read from a hard disk and sent over the network, the data may be copied four times: Read data from the hard disk to the kernel buffer. Copy from kernel buffer to user space buffer. Copy from user space buffer to kernel network buffer. Sent from the kernel network buffer to the network.

Zero copy technology reduces or eliminates these copying operations by:
- ** 'RMW_IMPLEMENTATION=rmw_fastrtps_cpp' ** : Ensure that the implementation is implemented using middleware that supports zero-copy transfer.
- ** 'FASTRTPS_DEFAULT_PROFILES_FILE' ** : Enable shared memory transporters through configuration files.
- ** 'RMW_FASTRTPS_USE_QOS_FROM_XML=1' ** : Ensure that QoS Settings are read from XML files to optimize transfer performance.
- ** 'ROS_DISABLE_LOANED_MESSAGES=0' ** : Enables the borrowed message mechanism to implement zero copy data transmission.

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/pzc/dev_ws/src/hbmem_pubsub/shm_fastdds.xml
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
export ROS_DISABLE_LOANED_MESSAGES=0
```

















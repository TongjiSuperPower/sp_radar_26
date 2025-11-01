#include <deque>
#include <queue>

namespace tools {
    
template <typename T, typename Container = std::deque<T>>
class IterableQueue : public std::queue<T, Container> {
public:
    IterableQueue(int max_size) {
        this->c = Container(); // 初始化底层容器
        max_size_ = max_size;
    }
    
    void push_back(const T &value) {
        if (this->c.size() >= max_size_) {
            this->c.pop_front(); // 如果超过最大大小，移除最前面的元素
        }
        this->c.push_back(value); // 使用底层容器的 push_back 方法
    }

    void push_front(const T &value) {
        if (this->c.size() >= max_size_) {
            this->c.pop_back(); // 如果超过最大大小，移除最后面的元素
        }
        this->c.push_front(value); // 使用底层容器的 push_front 方法
    }

    T& back() {
        return this->c.back(); // 返回队列的第一个元素
    }

    // 提供 begin() 和 end() 方法以支持范围-based for 循环
    auto begin() {
        return this->c.begin(); // c 是 std::queue 的保护成员，表示底层容器
    }
    
    auto end() {
        return this->c.end();
    }
    
    // const 版本
    auto begin() const {
        return this->c.begin();
    }
    
    auto end() const {
        return this->c.end();
    }
private:
    int max_size_;
};

}
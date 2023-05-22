#include "frame.h"

void Context::set_strategy(std::unique_ptr<Strategy> &&strategy)
{
        this->strategy_ = std::move(strategy);
}

void Context::doSomeBusinessLogic() const
{
        if (this->strategy_) {
            std::cout << "Context: Sorting data using the strategy (not sure how it'll do it)\n";
            std::string result = this->strategy_->doAlgorithm("aecbd");
            std::cout << result << "\n";
        } else {
            std::cout << "Context: Strategy isn't set\n";
        }
}

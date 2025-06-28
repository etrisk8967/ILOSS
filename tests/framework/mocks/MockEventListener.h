#pragma once

#include "core/external/TestWrapper.h"
#include "core/events/EventListener.h"
#include "core/events/Event.h"
#include <vector>
#include <memory>
#include <functional>

namespace iloss {
namespace test {

/**
 * @brief Mock event listener for testing event system
 */
class MockEventListener : public events::EventListener {
public:
    struct ReceivedEvent {
        std::unique_ptr<events::Event> event;
        std::chrono::system_clock::time_point receiveTime;
    };
    
    MockEventListener() = default;
    
    bool onEvent(events::Event& event) override {
        ReceivedEvent received;
        received.event = event.clone();
        received.receiveTime = std::chrono::system_clock::now();
        m_receivedEvents.push_back(std::move(received));
        
        // Call custom handler if set
        if (m_customHandler) {
            return m_customHandler(event);
        }
        
        // Return configured response
        return m_shouldHandleEvent;
    }
    
    // Test utilities
    void clear() {
        m_receivedEvents.clear();
    }
    
    size_t getEventCount() const { return m_receivedEvents.size(); }
    
    size_t getEventCount(events::EventCategory category) const {
        return std::count_if(m_receivedEvents.begin(), m_receivedEvents.end(),
            [category](const ReceivedEvent& r) { 
                return r.event->getCategory() == category; 
            });
    }
    
    size_t getEventCount(const std::string& eventType) const {
        return std::count_if(m_receivedEvents.begin(), m_receivedEvents.end(),
            [&eventType](const ReceivedEvent& r) { 
                return r.event->getType() == eventType; 
            });
    }
    
    const ReceivedEvent* getLastEvent() const {
        return m_receivedEvents.empty() ? nullptr : &m_receivedEvents.back();
    }
    
    const ReceivedEvent* getEvent(size_t index) const {
        return index < m_receivedEvents.size() ? &m_receivedEvents[index] : nullptr;
    }
    
    template<typename EventType>
    const EventType* getLastEventAs() const {
        auto last = getLastEvent();
        return last ? dynamic_cast<const EventType*>(last->event.get()) : nullptr;
    }
    
    bool hasReceivedEvent(const std::string& eventType) const {
        return getEventCount(eventType) > 0;
    }
    
    // Configuration
    void setShouldHandleEvent(bool handle) { m_shouldHandleEvent = handle; }
    
    void setCustomHandler(std::function<bool(events::Event&)> handler) {
        m_customHandler = handler;
    }
    
    // Wait for event with timeout
    bool waitForEvent(const std::string& eventType, 
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(1000)) {
        auto start = std::chrono::steady_clock::now();
        
        while (std::chrono::steady_clock::now() - start < timeout) {
            if (hasReceivedEvent(eventType)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        return false;
    }
    
    // Verification helpers
    void expectEventCount(size_t expected) const {
        EXPECT_EQ(getEventCount(), expected) 
            << "Expected " << expected << " events, but received " << getEventCount();
    }
    
    void expectEventType(const std::string& expectedType) const {
        auto last = getLastEvent();
        ASSERT_NE(last, nullptr) << "No events received";
        EXPECT_EQ(last->event->getType(), expectedType)
            << "Expected event type '" << expectedType 
            << "' but got '" << last->event->getType() << "'";
    }
    
    template<typename EventType>
    void expectLastEventIs() const {
        auto last = getLastEvent();
        ASSERT_NE(last, nullptr) << "No events received";
        EXPECT_NE(dynamic_cast<const EventType*>(last->event.get()), nullptr)
            << "Last event is not of expected type";
    }
    
private:
    std::vector<ReceivedEvent> m_receivedEvents;
    bool m_shouldHandleEvent = true;
    std::function<bool(events::Event&)> m_customHandler;
};

/**
 * @brief Mock event dispatcher for isolated testing
 */
class MockEventDispatcher {
public:
    void registerListener(std::shared_ptr<events::EventListener> listener,
                         events::EventCategory category = events::EventCategory::All) {
        m_listeners[category].push_back(listener);
    }
    
    void dispatch(std::unique_ptr<events::Event> event) {
        if (!event) return;
        
        auto category = event->getCategory();
        
        // Notify listeners for specific category
        auto it = m_listeners.find(category);
        if (it != m_listeners.end()) {
            for (auto& listener : it->second) {
                if (listener->onEvent(*event)) {
                    break;  // Event handled
                }
            }
        }
        
        // Notify listeners for all categories
        it = m_listeners.find(events::EventCategory::All);
        if (it != m_listeners.end()) {
            for (auto& listener : it->second) {
                if (listener->onEvent(*event)) {
                    break;  // Event handled
                }
            }
        }
    }
    
    void clear() {
        m_listeners.clear();
    }
    
    size_t getListenerCount(events::EventCategory category = events::EventCategory::All) const {
        auto it = m_listeners.find(category);
        return it != m_listeners.end() ? it->second.size() : 0;
    }
    
private:
    std::unordered_map<events::EventCategory, 
                      std::vector<std::shared_ptr<events::EventListener>>> m_listeners;
};

} // namespace test
} // namespace iloss
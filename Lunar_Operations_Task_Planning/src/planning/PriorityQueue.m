classdef PriorityQueue < handle
    properties (Access = private)
        elements   % Array to store elements
        priorities % Array to store priorities
        size      % Current size of queue
    end
    
    methods
        function obj = PriorityQueue()
            obj.elements = {};
            obj.priorities = [];
            obj.size = 0;
        end
        
        function insert(obj, element, priority)
            % Insert element with priority
            obj.size = obj.size + 1;
            obj.elements{obj.size} = element;
            obj.priorities(obj.size) = priority;
            obj.bubble_up(obj.size);
        end
        
        function [element, priority] = pop(obj)
            % Remove and return element with lowest priority
            if obj.size == 0
                element = [];
                priority = [];
                return;
            end
            
            element = obj.elements{1};
            priority = obj.priorities(1);
            
            obj.elements{1} = obj.elements{obj.size};
            obj.priorities(1) = obj.priorities(obj.size);
            
            obj.size = obj.size - 1;
            if obj.size > 0
                obj.bubble_down(1);
            end
        end
        
        function empty = isempty(obj)
            empty = (obj.size == 0);
        end
    end
    
    methods (Access = private)
        function bubble_up(obj, idx)
            % Move element up the heap if priority is lower than parent
            while idx > 1
                parent_idx = floor(idx/2);
                if obj.priorities(idx) < obj.priorities(parent_idx)
                    % Swap elements and priorities
                    [obj.elements{idx}, obj.elements{parent_idx}] = deal(obj.elements{parent_idx}, obj.elements{idx});
                    [obj.priorities(idx), obj.priorities(parent_idx)] = deal(obj.priorities(parent_idx), obj.priorities(idx));
                    idx = parent_idx;
                else
                    break;
                end
            end
        end
        
        function bubble_down(obj, idx)
            % Move element down the heap if priority is higher than children
            while true
                min_idx = idx;
                left = 2*idx;
                right = 2*idx + 1;
                
                if left <= obj.size && obj.priorities(left) < obj.priorities(min_idx)
                    min_idx = left;
                end
                if right <= obj.size && obj.priorities(right) < obj.priorities(min_idx)
                    min_idx = right;
                end
                
                if min_idx ~= idx
                    % Swap elements and priorities
                    [obj.elements{idx}, obj.elements{min_idx}] = deal(obj.elements{min_idx}, obj.elements{idx});
                    [obj.priorities(idx), obj.priorities(min_idx)] = deal(obj.priorities(min_idx), obj.priorities(idx));
                    idx = min_idx;
                else
                    break;
                end
            end
        end
    end
end

function text = format_scalar(value)
if islogical(value)
    text = string(double(value));
    text = char(text);
    return;
end

if isnumeric(value)
    text = sprintf('%.15g', value);
    return;
end

text = char(string(value));
end

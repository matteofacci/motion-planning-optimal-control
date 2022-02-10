function val = value_from_user(prompt,default)

val = input(prompt);
if isempty(val)
    val = default;
end

end
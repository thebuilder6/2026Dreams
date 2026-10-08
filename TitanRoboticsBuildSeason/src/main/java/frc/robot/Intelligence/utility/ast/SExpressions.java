package frc.robot.Intelligence.utility.ast;

import java.util.ArrayList;
import java.util.List;

/**
 * Text codec for {@link ExpressionNode} genomes — the genome is a string, so a
 * search harness in any language can emit a candidate and Java can evaluate it.
 *
 * <p>The wire form is the same pseudo-code {@link ExpressionNode#toReadableString}
 * produces, e.g.
 * <pre>
 *   product(my_hub_active, sigmoid(held_ratio, 6, 0.27), clamp(dist_to_hub, 0.4, 1))
 * </pre>
 * Numbers are constants; bare identifiers are terminals; {@code name(...)} is an
 * operator. Unknown names and wrong arities are rejected, so a malformed genome
 * fails loudly instead of evaluating to something plausible.
 */
public final class SExpressions {

    private SExpressions() {}

    /** Serializes a genome to its canonical one-line form. */
    public static String write(ExpressionNode node) {
        if (node == null) {
            throw new IllegalArgumentException("node must not be null");
        }
        return node.toReadableString();
    }

    /** Parses a genome produced by {@link #write} (or hand-written to the grammar). */
    public static ExpressionNode read(String text) {
        if (text == null || text.isBlank()) {
            throw new IllegalArgumentException("empty genome");
        }
        Parser parser = new Parser(text);
        ExpressionNode node = parser.parseExpr();
        if (parser.peek() != null) {
            throw new IllegalArgumentException("trailing tokens after genome: '" + parser.peek().text + "'");
        }
        return node;
    }

    /** Rejects a genome that reads a clairvoyant-only terminal (deploy safety). */
    public static boolean usesClairvoyantTerminal(ExpressionNode node) {
        if (node instanceof ExpressionNode.TerminalNode t) {
            return t.terminal().tier() == Terminal.Tier.CLAIRVOYANT;
        }
        for (ExpressionNode child : node.children()) {
            if (usesClairvoyantTerminal(child)) {
                return true;
            }
        }
        return false;
    }

    // ------------------------------------------------------------------

    private record Token(String text, boolean number) {}

    private static final class Parser {
        private final List<Token> tokens = new ArrayList<>();
        private int pos;

        Parser(String text) {
            tokenize(text);
        }

        private void tokenize(String text) {
            int i = 0;
            int n = text.length();
            while (i < n) {
                char c = text.charAt(i);
                if (Character.isWhitespace(c)) {
                    i++;
                } else if (c == '(' || c == ')' || c == ',') {
                    tokens.add(new Token(String.valueOf(c), false));
                    i++;
                } else if (c == '-' || c == '+' || c == '.' || Character.isDigit(c)) {
                    int start = i;
                    i++;
                    while (i < n && (Character.isDigit(text.charAt(i)) || text.charAt(i) == '.'
                            || text.charAt(i) == 'e' || text.charAt(i) == 'E'
                            || ((text.charAt(i) == '-' || text.charAt(i) == '+')
                                    && (text.charAt(i - 1) == 'e' || text.charAt(i - 1) == 'E')))) {
                        i++;
                    }
                    tokens.add(new Token(text.substring(start, i), true));
                } else if (Character.isLetter(c) || c == '_') {
                    int start = i;
                    i++;
                    while (i < n && (Character.isLetterOrDigit(text.charAt(i)) || text.charAt(i) == '_')) {
                        i++;
                    }
                    tokens.add(new Token(text.substring(start, i), false));
                } else {
                    throw new IllegalArgumentException("unexpected character '" + c + "' in genome");
                }
            }
        }

        Token peek() {
            return pos < tokens.size() ? tokens.get(pos) : null;
        }

        private Token next() {
            if (pos >= tokens.size()) {
                throw new IllegalArgumentException("unexpected end of genome");
            }
            return tokens.get(pos++);
        }

        private void expect(String text) {
            Token t = next();
            if (!t.text.equals(text)) {
                throw new IllegalArgumentException("expected '" + text + "' but found '" + t.text + "'");
            }
        }

        ExpressionNode parseExpr() {
            Token t = next();
            if (t.number) {
                return new ExpressionNode.Constant(Double.parseDouble(t.text));
            }
            String name = t.text;
            if (peek() != null && peek().text.equals("(")) {
                expect("(");
                List<ExpressionNode> args = new ArrayList<>();
                if (peek() != null && !peek().text.equals(")")) {
                    args.add(parseExpr());
                    while (peek() != null && peek().text.equals(",")) {
                        expect(",");
                        args.add(parseExpr());
                    }
                }
                expect(")");
                return buildOp(name, args);
            }
            Terminal terminal = Terminal.fromName(name);
            if (terminal == null) {
                throw new IllegalArgumentException("unknown terminal or operator '" + name + "'");
            }
            return new ExpressionNode.TerminalNode(terminal);
        }

        private ExpressionNode buildOp(String name, List<ExpressionNode> args) {
            return switch (name.toLowerCase(java.util.Locale.ROOT)) {
                case "product" -> new ExpressionNode.Product(args);
                case "sum" -> new ExpressionNode.Sum(args);
                case "min" -> new ExpressionNode.Min(args);
                case "max" -> new ExpressionNode.Max(args);
                case "if" -> {
                    require(args, 3, name);
                    yield new ExpressionNode.IfThenElse(args.get(0), args.get(1), args.get(2));
                }
                case "threshold" -> {
                    require(args, 4, name);
                    yield new ExpressionNode.Threshold(args.get(0), num(args.get(1), name),
                            num(args.get(2), name), num(args.get(3), name));
                }
                case "threshold_ge" -> {
                    require(args, 4, name);
                    yield new ExpressionNode.Threshold(args.get(0), num(args.get(1), name),
                            num(args.get(2), name), num(args.get(3), name), true);
                }
                case "scale" -> {
                    require(args, 2, name);
                    yield new ExpressionNode.Scale(args.get(0), num(args.get(1), name));
                }
                case "sigmoid" -> {
                    require(args, 3, name);
                    yield new ExpressionNode.Sigmoid(args.get(0), num(args.get(1), name), num(args.get(2), name));
                }
                case "gaussian" -> {
                    require(args, 3, name);
                    yield new ExpressionNode.Gaussian(args.get(0), num(args.get(1), name), num(args.get(2), name));
                }
                case "pow" -> {
                    require(args, 2, name);
                    yield new ExpressionNode.Power(args.get(0), num(args.get(1), name));
                }
                case "clamp" -> {
                    require(args, 3, name);
                    yield new ExpressionNode.Clamp(args.get(0), num(args.get(1), name), num(args.get(2), name));
                }
                case "not" -> {
                    require(args, 1, name);
                    yield new ExpressionNode.Not(args.get(0));
                }
                default -> throw new IllegalArgumentException("unknown operator '" + name + "'");
            };
        }

        private static void require(List<ExpressionNode> args, int arity, String name) {
            if (args.size() != arity) {
                throw new IllegalArgumentException(name + " expects " + arity + " arguments, got " + args.size());
            }
        }

        private static double num(ExpressionNode node, String op) {
            if (node instanceof ExpressionNode.Constant c) {
                return c.value();
            }
            throw new IllegalArgumentException(op + " expects a numeric constant argument");
        }
    }
}

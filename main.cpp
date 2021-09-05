#include <bits/stdc++.h>
using namespace std;
#define rep(i, n) for (int i = 0; i < (int)(n); i++)
#define ALL(v) (v).begin(), (v).end()
using ll = long long;
using P = pair<int, int>;

constexpr double TIMELIMIT = 2.95;
constexpr int dx[] = {0, 1, 0, -1}, dy[] = {1, 0, -1, 0};

int N;
vector<string> c;
int sx, sy;

string ans;
string rev;

map<int, char> mp;

vector<P> inters;
vector<vector<int>> intersV;
int V = 0;
vector<vector<P>> g;
vector<vector<P>> G;
vector<vector<string>> moves;
vector<vector<int>> prevv;
vector<vector<int>> dist;

template <typename T>
vector<T> Dijkstra(int s, vector<vector<pair<int, T>>>& G, vector<int>& prev) {
    const T INF = numeric_limits<T>::max();
    using P = pair<T, int>;
    int V = G.size();
    vector<T> dist(V, INF);
    priority_queue<P, vector<P>, greater<P>> que;
    dist[s] = 0;
    que.emplace(0, s);
    prev.assign(V, -1);
    while (!que.empty()) {
        P p = que.top();
        que.pop();
        int v = p.second;
        if (dist[v] < p.first) continue;
        for (int i = 0; i < (int)G[v].size(); i++) {
            int to = G[v][i].first;
            T cost = G[v][i].second;
            if (dist[to] > dist[v] + cost) {
                dist[to] = dist[v] + cost;
                prev[to] = v;
                que.emplace(dist[to], to);
            }
        }
    }
    return dist;
}

struct XorShift {
    unsigned int x, y, z, w, t;

    XorShift(int seed) {
        mt19937 rnd(seed);
        x = rnd();
        y = rnd();
        z = rnd();
        w = rnd();
        t = 1;
    }

    int rand() {
        t = x ^ (x << 11);
        x = y;
        y = z;
        z = w;
        w = (w ^ (w >> 19)) ^ (t ^ (t >> 8));
        return w & 0x7fffffff;
    }
} rnd(rand());

struct Timer {
    chrono::system_clock::time_point start, now;

    Timer() {
        start = chrono::system_clock::now();
    }

    double getTime() {
        now = chrono::system_clock::now();
        return chrono::duration<double>(now - start).count();
    }
};

Timer tmr;
double nowclock, startclock;

void input() {
    cin >> N >> sx >> sy;
    c.resize(N);
    rep(i, N) {
        cin >> c[i];
    }
}

int f(int x, int y) {
    return x * N + y;
}

P h(int v) {
    return P(v / N, v % N);
}

void interEnum() {
    g.resize(N * N);
    vector<P> ti;
    bool tiv[N][N];
    fill(tiv[0], tiv[N], false);
    rep(x, N) {
        rep(y, N) {
            if (c[x][y] == '#') continue;
            int cnt[2] = {};
            rep(i, 4) {
                int nx = x + dx[i], ny = y + dy[i];
                if (nx < 0 || N <= nx || ny < 0 || N <= ny) continue;
                if (c[nx][ny] == '#') continue;
                g[f(x, y)].emplace_back(f(nx, ny), c[nx][ny] - '0');
                cnt[i % 2]++;
            }
            if (cnt[0] && cnt[1]) {
                ti.emplace_back(x, y);
                tiv[x][y] = true;
            }
        }
    }
    intersV.resize(N, vector<int>(N, -1));
    for (P p : ti) {
        int x = p.first, y = p.second;
        int cnt[2] = {};
        rep(i, 4) {
            int nx = x + dx[i], ny = y + dy[i];
            while (!(nx < 0 || N <= nx || ny < 0 || N <= ny) &&
                   c[nx][ny] != '#') {
                if (tiv[nx][ny]) {
                    cnt[i % 2]++;
                    break;
                }
                nx += dx[i];
                ny += dy[i];
            }
        }
        if (cnt[0] && cnt[1]) {
            tiv[x][y] = false;
        } else {
            inters.emplace_back(x, y);
            intersV[x][y] = V++;
        }
    }
    G.resize(V);
    moves.resize(V, vector<string>(V));
    for (P p : inters) {
        int x = p.first, y = p.second;
        vector<int> prev;
        auto dist = Dijkstra(f(x, y), g, prev);
        for (P pp : inters) {
            int nx = pp.first, ny = pp.second;
            if (x == nx && y == ny) continue;
            G[intersV[x][y]].emplace_back(intersV[nx][ny], dist[f(nx, ny)]);
            string t;
            int curx = nx, cury = ny;
            while (prev[f(curx, cury)] != -1) {
                P p = h(prev[f(curx, cury)]);
                if (p.first - 1 == curx) {
                    t.push_back('U');
                } else if (p.first + 1 == curx) {
                    t.push_back('D');
                } else if (p.second - 1 == cury) {
                    t.push_back('L');
                } else {
                    t.push_back('R');
                }
                tie(curx, cury) = p;
            }
            reverse(ALL(t));
            moves[intersV[x][y]][intersV[nx][ny]] = t;
        }
    }
    dist.resize(V);
    for (int v = 0; v < V; v++) {
        vector<int> t;
        dist[v] = Dijkstra(v, G, t);
    }
}

int nearInter() {
    if (intersV[sx][sy] != -1) return intersV[sx][sy];
    int mn = INT_MAX;
    int res = -1;
    vector<int> prev;
    auto dist = Dijkstra(f(sx, sy), g, prev);
    for (P p : inters) {
        int nx = p.first, ny = p.second;
        if (dist[f(nx, ny)] < mn) {
            mn = dist[f(nx, ny)];
            res = intersV[nx][ny];
            string s, t;
            int curx = nx, cury = ny;
            while (prev[f(curx, cury)] != -1) {
                P p = h(prev[f(curx, cury)]);
                if (p.first - 1 == curx) {
                    s.push_back('U');
                    t.push_back('D');
                } else if (p.first + 1 == curx) {
                    s.push_back('D');
                    t.push_back('U');
                } else if (p.second - 1 == cury) {
                    s.push_back('L');
                    t.push_back('R');
                } else {
                    s.push_back('R');
                    t.push_back('L');
                }
                tie(curx, cury) = p;
            }
            reverse(ALL(s));
            ans = s;
            rev = t;
        }
    }
    return res;
}

struct State {
    int score;
    vector<int> order;
};

void init(int start, State& state) {
    state.score = 0;
    bool visited[V] = {};
    int cur = start;
    state.order.emplace_back(cur);
    visited[cur] = true;
    while(true) {
        int mn = INT_MAX;
        int to = -1;
        rep(v, V) {
            if (!visited[v] && dist[cur][v] < mn) {
                mn = dist[cur][v];
                to = v;
            }
        }
        if (to == -1) break;
        state.order.emplace_back(to);
        state.score += mn;
        visited[to] = true;
        cur = to;
    }
    state.order.emplace_back(start);
    state.score += dist[cur][start];
}

void calc(State& state) {
    state.score = 0;
    rep(i, state.order.size() - 1) {
        state.score += dist[state.order[i]][state.order[i + 1]];
    }
}

void modify(State& state) {
    int x = rnd.rand() % (state.order.size() - 2);
    int y = rnd.rand() % (state.order.size() - 2);
    while (y == x) {
        y = rnd.rand() % (state.order.size() - 2);
    }
    x++, y++;
    reverse(state.order.begin() + x, state.order.begin() + y);
    calc(state);
}

void solve(State& state) {
    int steps = 0;
    double starttemp = 50, endtemp = 0;
    while (true) {
        if (steps % 10000 == 0) {
            nowclock = tmr.getTime();
            if (nowclock - startclock > TIMELIMIT) break;
        }
        State newstate = state;
        modify(newstate);
        double temp = starttemp + (endtemp - starttemp) *
                                      (nowclock - startclock) / TIMELIMIT;
        double prob = exp((state.score - newstate.score) / temp);
        if (prob > (rnd.rand() % (int)1e9) / 1e9) {
            state = newstate;
        }
        steps++;
    }
    cerr << "score : " << state.score << endl;
    cerr << "steps : " << steps << endl;
}

void toString(State& state) {
    rep(i, state.order.size() - 1) {
        ans += moves[state.order[i]][state.order[i + 1]];
    }
}

void output() {
    cout << ans << endl;
}

signed main() {
    startclock = tmr.getTime();
    mp[0] = 'R';
    mp[1] = 'D';
    mp[2] = 'L';
    mp[3] = 'U';
    input();
    interEnum();
    int start = nearInter();
    State state;
    init(start, state);
    solve(state);
    toString(state);
    ans += rev;
    output();
    return 0;
}
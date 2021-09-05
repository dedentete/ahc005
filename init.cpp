#include <bits/stdc++.h>
using namespace std;
#define rep(i, n) for (int i = 0; i < (int)(n); i++)
#define ALL(v) (v).begin(), (v).end()
using ll = long long;
using P = pair<int, int>;

constexpr double TIMELIMIT = 2.9;
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

void interEnum() {
    intersV.resize(N, vector<int>(N, -1));
    rep(x, N) {
        rep(y, N) {
            if (c[x][y] == '#') continue;
            int cnt[2] = {};
            rep(i, 4) {
                int nx = x + dx[i], ny = y + dy[i];
                if (nx < 0 || N <= nx || ny < 0 || N <= ny) continue;
                if (c[nx][ny] == '#') continue;
                cnt[i % 2]++;
            }
            if (cnt[0] && cnt[1]) {
                inters.emplace_back(x, y);
                intersV[x][y] = V++;
            }
        }
    }
    G.resize(V);
    moves.resize(V, vector<string>(V));
    for (P p : inters) {
        int x = p.first, y = p.second;
        rep(i, 4) {
            int nx = x + dx[i], ny = y + dy[i];
            int dist = 0;
            string s;
            while (!(nx < 0 || N <= nx || ny < 0 || N <= ny) &&
                   c[nx][ny] != '#') {
                dist += c[nx][ny] - '0';
                s.push_back(mp[i]);
                if (intersV[nx][ny] != -1) {
                    G[intersV[x][y]].emplace_back(intersV[nx][ny], dist);
                    moves[intersV[x][y]][intersV[nx][ny]] = s;
                }
                nx += dx[i];
                ny += dy[i];
            }
        }
    }
    dist.resize(V);
    prevv.resize(V);
    for (int v = 0; v < V; v++) {
        dist[v] = Dijkstra(v, G, prevv[v]);
    }
}

int nearInter() {
    if (intersV[sx][sy] != -1) return intersV[sx][sy];
    int mn = INT_MAX;
    int res = -1;
    rep(i, 4) {
        int nx = sx + dx[i], ny = sy + dy[i];
        int dist = 0;
        string s, t;
        while (!(nx < 0 || N <= nx || ny < 0 || N <= ny) && c[nx][ny] != '#') {
            dist += c[nx][ny] - '0';
            s.push_back(mp[i]);
            t.push_back(mp[(i + 2) % 4]);
            if (intersV[nx][ny] != -1 && dist < mn) {
                dist = mn;
                res = intersV[nx][ny];
                ans = s;
                rev = t;
            }
            nx += dx[i];
            ny += dy[i];
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
    rep(_, V - 1) {
        int mn = INT_MAX;
        int to = -1;
        rep(v, V) {
            if (!visited[v] && dist[cur][v] < mn) {
                mn = dist[cur][v];
                to = v;
            }
        }
        state.order.emplace_back(to);
        state.score += mn;
        visited[to] = true;
        cur = to;
    }
    state.order.emplace_back(start);
    state.score += dist[cur][start];
}

void toString(State& state) {
    rep(i, state.order.size() - 1) {
        vector<int> v;
        int cur = state.order[i + 1];
        v.emplace_back(cur);
        while (prevv[state.order[i]][cur] != -1) {
            cur = prevv[state.order[i]][cur];
            v.emplace_back(cur);
        }
        reverse(ALL(v));
        rep(i, v.size() - 1) {
            ans += moves[v[i]][v[i + 1]];
        }
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
    toString(state);
    ans += rev;
    output();
    return 0;
}